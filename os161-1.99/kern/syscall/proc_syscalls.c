#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <kern/fcntl.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include <machine/trapframe.h>
#include <limits.h>
#include <vfs.h>

#include "opt-A2.h"

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  (void)exitcode;

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);
#if OPT_A2
  p->exit_status = _MKWAIT_EXIT(exitcode);
#endif

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */

  proc_remthread(curthread);
  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);
 
  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
#if OPT_A2
    *retval = curproc->pid;
#else
  *retval = 1;
#endif
  return(0);
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
    int exitstatus = 0;
    int result;

    /* this is just a stub implementation that always reports an
       exit status of 0, regardless of the actual exit status of
       the specified process.   
       In fact, this will return 0 even if the specified process
       is still running, and even if it never existed in the first place.

       Fix this!
       */

    if (options != 0) {
        return(EINVAL);
    }

    if (status == NULL) {
        return EFAULT;
    }

    struct proc *candidate = get_proc_by_pid(pid);

    if (candidate == NULL) { // does not exist
        return ESRCH;
    }

    if (candidate->parent_pid != curproc->pid) {
        return ECHILD;
    }

    lock_acquire(candidate->proc_lock);
    if (is_proc_alive(pid)) {
        cv_wait(candidate->proc_cv, candidate->proc_lock);
    }
    lock_release(candidate->proc_lock);

    exitstatus = candidate->exit_status;

    result = copyout((void *)&exitstatus,status,sizeof(int));
    if (result) {
        return(result);
    }
    *retval = pid;
    return(0);
}

#if OPT_A2
int
sys_fork(struct trapframe *tf, pid_t *retval) {
    struct proc *new_proc = proc_create_runprogram("");
    if (new_proc == NULL) {
        return ENOMEM;
    }
    int *retval_pid = kmalloc(sizeof(int*));
    get_pid_counter(retval_pid);
    if (*retval_pid == 65) {
        proc_destroy(new_proc);
        return ENPROC;
    }
    kfree(retval_pid);
    // Copy address space
    struct addrspace *new_addrspace;
    int copy_addrspace = as_copy(curproc->p_addrspace, &new_addrspace);
    if (copy_addrspace != 0) {
        proc_destroy(new_proc);
        return copy_addrspace;
    }
    if (new_addrspace == NULL) {
        proc_destroy(new_proc);
        return ENOMEM;
    }
    spinlock_acquire(&new_proc->p_lock);
    new_proc->p_addrspace = new_addrspace;
    spinlock_release(&new_proc->p_lock);
    // Set parent-child relationship
    new_proc->parent_pid = curproc->pid;
    int add_child = array_add(curproc->children, (void *)new_proc, NULL);
    if (add_child != 0) {
        proc_destroy(new_proc);
        return ENOMEM;
    }
    // Create thread for child process
    struct trapframe *parent_tf = kmalloc(sizeof(struct trapframe));
    *parent_tf = *tf;
    int fork_thread = thread_fork(new_proc->p_name, new_proc, enter_forked_process, parent_tf, 0);
    if (fork_thread != 0) {
        kfree(parent_tf);
        proc_destroy(new_proc);
        return ENOMEM;
    }
    *retval = new_proc->pid;

    return 0;
}

int
sys_execv(userptr_t program, userptr_t args) {
    struct addrspace *as;
    struct addrspace *old_as;
    struct vnode *v;
    vaddr_t entrypoint, stackptr;
    int result = 0;
    int assigned = 0; // args in argv
    char **arg_array = (char **)args;

    /* Count args */
    int argc = 0;
    char *arg = arg_array[argc];
    while(arg) {
        argc += 1;
        arg = arg_array[argc];
    }

    if (argc * 1024 > ARG_MAX) {
        return E2BIG;
    }
    // for progname
    argc += 1;

    /* Copy the program path into the kernel */
    char progname[PATH_MAX];
    size_t actual;
    result = copyinstr((const_userptr_t)program, progname, PATH_MAX, &actual);
    if (result) {
        return result;
    }
    if (actual <= 1) {
        // program name is only a null terminator
        return ENOENT;
    }

    char **argv = kmalloc((argc+1)*sizeof(char *));
    if (argv == NULL) {
        return ENOMEM;
    }
   
    argv[0] = (char *)kmalloc((actual)*sizeof(char));
    if (argv[0] == NULL) {
        kprintf("No memory to copy argv[0]\n");
        kfree(argv);
        return ENOMEM;
    }
    result = copyinstr((const_userptr_t)program, argv[0], NAME_MAX+1, &actual);
    if (result) {
        kprintf("Couldn't copyinstr the progname\n");
        kfree(argv[0]);
        kfree(argv);
        return result;
    }
    assigned += 1;
    for (int i = 1; i < argc; ++i) {
        argv[i] = (char *)kmalloc((strlen(arg_array[i-1])+1)*sizeof(char));
        if (argv[i] == NULL) {
            kprintf("No memory to copy argv[%d]\n", i);
            for (int j = 0; j < assigned; ++j) {
                kfree(argv[j]);
            }
            kfree(argv);
            return ENOMEM;
        }
        result = copyinstr((const_userptr_t)arg_array[i-1], argv[i], NAME_MAX+1, &actual);
        if (result) {
            kprintf("Couldn't copyinstr the arg to argv[%d]\n", i);
            for (int j = 0; j < assigned; ++j) {
                kfree(argv[j]);
            }
            kfree(argv);
            return result;
        }
        assigned += 1;
    }
    argv[argc] = NULL;
    assigned += 1;

   /* Open the file. */
    char *fname_temp;
    fname_temp = kstrdup(progname);
    result = vfs_open(fname_temp, O_RDONLY, 0, &v);
    kfree(fname_temp);
    if (result) {
        for (int j = 0; j < assigned; ++j) {
            kfree(argv[j]);
        }
        kfree(argv);
        return result;
    }

    /* Create a new address space. */
    as = as_create();
    if (as == NULL) {
        for (int j = 0; j < assigned; ++j) {
            kfree(argv[j]);
        }
        kfree(argv);
        vfs_close(v);
        return ENOMEM;
    }

    /* Switch to it and activate it. */
    as_deactivate();
    old_as = curproc_setas(as);
    as_activate();

    /* Load the executable. */
    result = load_elf(v, &entrypoint);
    if (result) {
        for (int j = 0; j < assigned; ++j) {
            kfree(argv[j]);
        }
        kfree(argv);
        /* p_addrspace will go away when curproc is destroyed */
        vfs_close(v);
        return result;
    }

    /* Done with the file now. */
    vfs_close(v);

    /* Define the user stack in the address space */
    result = as_define_stack(as, &stackptr, argc, argv);
    if (result) {
        for (int j = 0; j < assigned; ++j) {
            kfree(argv[j]);
        }
        kfree(argv);
        /* p_addrspace will go away when curproc is destroyed */
        return result;
    }

    // Destroy old addrspace
    as_destroy(old_as);

    // Free memory
    for (int j = 0; j < assigned; ++j) {
        kfree(argv[j]);
    }
    kfree(argv);

    /* Warp to user mode. */
    enter_new_process(argc /*argc*/, (userptr_t)stackptr /*userspace addr of argv*/, stackptr, entrypoint);

    /* enter_new_process does not return. */
    panic("enter_new_process returned\n");
    return EINVAL;

}
#endif

