#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */

bool is_right_turn(Direction origin, Direction destination) {
    if (origin == north) {
        return destination == west;
    } else if (origin == west) {
        return destination == south;
    } else if (origin == south) {
        return destination == east;
    } else if (origin == east) {
        return destination == north;
    } else {
        panic("invalid origin and/or destination\n");
        return false;
    }
}

bool is_left_turn(Direction origin, Direction destination) {
    if (origin == north) {
        return destination == east;
    } else if (origin == west) {
        return destination == north;
    } else if (origin == south) {
        return destination == west;
    } else if (origin == east) {
        return destination == south;
    } else {
        panic("invalid origin and/or destination\n");
        return false;
    }
}

int n_counter, s_counter, w_counter, e_counter; // waiting to go to a destination
int n_right_counter, s_right_counter, w_right_counter, e_right_counter; // waiting to go to a destination, turning right
int n_left_counter, s_left_counter, w_left_counter, e_left_counter; // waiting to go to a destination, turning left
struct lock *intersection_lock;
struct cv *n, *nl, *nr, *s, *sl, *sr, *w, *wl, *wr, *e, *el, *er; // destination

/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
    n_counter = 0;
    s_counter = 0;
    w_counter = 0;
    e_counter = 0;
    n_right_counter = 0;
    s_right_counter = 0;
    w_right_counter = 0;
    e_right_counter = 0;
    n_left_counter = 0;
    s_left_counter = 0;
    w_left_counter = 0;
    e_left_counter = 0;
    intersection_lock = lock_create("Intersection Lock");
    n = cv_create("North");
    nl = cv_create("North Left Turn");
    nr = cv_create("North Right Turn");
    s = cv_create("South");
    sl = cv_create("South Left Turn");
    sr = cv_create("South Right Turn");
    w = cv_create("West");
    wl = cv_create("West Left Turn");
    wr = cv_create("West Right Turn");
    e = cv_create("East");
    el = cv_create("East Left Turn");
    er = cv_create("East Right Turn");

    KASSERT(intersection_lock != NULL);
    KASSERT(n != NULL);
    KASSERT(nl != NULL);
    KASSERT(nr != NULL);
    KASSERT(s != NULL);
    KASSERT(sl != NULL);
    KASSERT(sr != NULL);
    KASSERT(w != NULL);
    KASSERT(wl != NULL);
    KASSERT(wr != NULL);
    KASSERT(e != NULL);
    KASSERT(el != NULL);
    KASSERT(er != NULL);
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
    KASSERT(intersection_lock != NULL);
    KASSERT(n != NULL);
    KASSERT(nl != NULL);
    KASSERT(nr != NULL);
    KASSERT(s != NULL);
    KASSERT(sl != NULL);
    KASSERT(sr != NULL);
    KASSERT(w != NULL);
    KASSERT(wl != NULL);
    KASSERT(wr != NULL);
    KASSERT(e != NULL);
    KASSERT(el != NULL);
    KASSERT(er != NULL);

    lock_destroy(intersection_lock);
    cv_destroy(n);
    cv_destroy(nl);
    cv_destroy(nr);
    cv_destroy(s);
    cv_destroy(sl);
    cv_destroy(sr);
    cv_destroy(w);
    cv_destroy(wl);
    cv_destroy(wr);
    cv_destroy(e);
    cv_destroy(el);
    cv_destroy(er);
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
    KASSERT(intersection_lock != NULL);
    KASSERT(n != NULL);
    KASSERT(nl != NULL);
    KASSERT(nr != NULL);
    KASSERT(s != NULL);
    KASSERT(sl != NULL);
    KASSERT(sr != NULL);
    KASSERT(w != NULL);
    KASSERT(wl != NULL);
    KASSERT(wr != NULL);
    KASSERT(e != NULL);
    KASSERT(el != NULL);
    KASSERT(er != NULL);

    lock_acquire(intersection_lock);
    if (destination == north) {
        if (is_right_turn(origin, destination)) {
            while(n_counter > 0 || n_left_counter > 0) {
                cv_wait(nr, intersection_lock);
            }
            n_right_counter++;
        } else if (is_left_turn(origin, destination)) {
            while(w_counter > 0 || n_counter > 0 || s_counter > 0 || n_right_counter > 0 || w_left_counter > 0 || s_left_counter > 0 || e_left_counter > 0) {
                cv_wait(nl, intersection_lock);
            }
            n_left_counter++;
        } else {
            while(w_counter > 0 || e_counter > 0 || n_right_counter > 0 || n_left_counter > 0 || s_left_counter > 0 || e_left_counter > 0) {
                cv_wait(n, intersection_lock);
            }
            n_counter++;
        }
    } else if (destination == south) {
        if (is_right_turn(origin, destination)) {
            while(s_counter > 0 || s_left_counter > 0) {
                cv_wait(sr, intersection_lock);
            }
            s_right_counter++;
        } else if (is_left_turn(origin, destination)) {
            while(e_counter > 0 || n_counter > 0 || s_counter > 0 || s_right_counter > 0 || w_left_counter > 0 || n_left_counter > 0 || e_left_counter > 0) {
                cv_wait(sl, intersection_lock);
            }
            s_left_counter++;
        } else {
            while(w_counter > 0 || e_counter > 0 || s_right_counter > 0 || n_left_counter > 0 || s_left_counter > 0 || w_left_counter > 0) {
                cv_wait(s, intersection_lock);
            }
            s_counter++;
        }
    } else if (destination == west) {
       if (is_right_turn(origin, destination)) {
            while(w_counter > 0 || w_left_counter > 0) {
                cv_wait(wr, intersection_lock);
            }
            w_right_counter++;
        } else if (is_left_turn(origin, destination)) {
            while(e_counter > 0 || w_counter > 0 || s_counter > 0 || w_right_counter > 0 || e_left_counter > 0 || s_left_counter > 0 || n_left_counter > 0) {
                cv_wait(wl, intersection_lock);
            }
            w_left_counter++;
        } else {
            while(n_counter > 0 || s_counter > 0 || w_right_counter > 0 || w_left_counter > 0 || n_left_counter > 0 || e_left_counter > 0) {
                cv_wait(w, intersection_lock);
            }
            w_counter++;
        }
    } else if (destination == east) {
       if (is_right_turn(origin, destination)) {
            while(e_counter > 0 || e_left_counter > 0) {
                cv_wait(er, intersection_lock);
            }
            e_right_counter++;
        } else if (is_left_turn(origin, destination)) {
            while(e_counter > 0 || w_counter > 0 || n_counter > 0 || e_right_counter > 0 || w_left_counter > 0 || s_left_counter > 0 || n_left_counter > 0) {
                cv_wait(el, intersection_lock);
            }
            e_left_counter++;
        } else {
            while(n_counter > 0 || s_counter > 0 || e_right_counter > 0 || w_left_counter > 0 || s_left_counter > 0 || e_left_counter > 0) {
                cv_wait(e, intersection_lock);
            }
            e_counter++;
        }
    }
    lock_release(intersection_lock);
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
    KASSERT(intersection_lock != NULL);
    KASSERT(n != NULL);
    KASSERT(nl != NULL);
    KASSERT(nr != NULL);
    KASSERT(s != NULL);
    KASSERT(sl != NULL);
    KASSERT(sr != NULL);
    KASSERT(w != NULL);
    KASSERT(wl != NULL);
    KASSERT(wr != NULL);
    KASSERT(e != NULL);
    KASSERT(el != NULL);
    KASSERT(er != NULL);

    lock_acquire(intersection_lock);
    if (destination == north) {
        if (is_right_turn(origin, destination)) {
            n_right_counter--;
        } else if (is_left_turn(origin, destination)) {
            n_left_counter--;
        } else {
            n_counter--;
        }
    } else if (destination == south) {
        if (is_right_turn(origin, destination)) {
            s_right_counter--;
        } else if (is_left_turn(origin, destination)) {
            s_left_counter--;
        } else {
            s_counter--;
        }
    } else if (destination == west) {
        if (is_right_turn(origin, destination)) {
            w_right_counter--;
        } else if (is_left_turn(origin, destination)) {
            w_left_counter--;
        } else {
            w_counter--;
        }
    } else if (destination == east) {
        if (is_right_turn(origin, destination)) {
            e_right_counter--;
        } else if (is_left_turn(origin, destination)) {
            e_left_counter--;
        } else {
            e_counter--;
        }
    }
    if (n_counter > 0 && n_left_counter > 0) {
        cv_signal(nr, intersection_lock);
    }
    if (w_counter > 0 && n_counter > 0 && s_counter > 0 && n_right_counter > 0 && w_left_counter > 0 && s_left_counter > 0 && e_left_counter > 0) {
        cv_signal(nl, intersection_lock);
    }
    if (w_counter > 0 && e_counter > 0 && n_right_counter > 0 && n_left_counter > 0 && s_left_counter > 0 && e_left_counter > 0) {
        cv_signal(n, intersection_lock);
    }
    if (s_counter > 0 && s_left_counter > 0) {
        cv_signal(sr, intersection_lock);
    }
    if (e_counter > 0 && n_counter > 0 && s_counter > 0 && s_right_counter > 0 && w_left_counter > 0 && n_left_counter > 0 && e_left_counter > 0) {
        cv_signal(sl, intersection_lock);
    if (w_counter > 0 && e_counter > 0 && s_right_counter > 0 && n_left_counter > 0 && s_left_counter > 0 && w_left_counter > 0) {
        cv_signal(s, intersection_lock);
    }
    if (w_counter > 0 && w_left_counter > 0) {
        cv_signal(wr, intersection_lock);
    if (e_counter > 0 && w_counter > 0 && s_counter > 0 && w_right_counter > 0 && e_left_counter > 0 && s_left_counter > 0 && n_left_counter > 0) {
        cv_signal(wl, intersection_lock);
    if (n_counter > 0 && s_counter > 0 && w_right_counter > 0 && w_left_counter > 0 && n_left_counter > 0 && e_left_counter > 0) {
        cv_signal(w, intersection_lock);
    }
    if (e_counter > 0 && e_left_counter > 0) {
        cv_signal(er, intersection_lock);
    }
    if (e_counter > 0 && w_counter > 0 && n_counter > 0 && e_right_counter > 0 && w_left_counter > 0 && s_left_counter > 0 && n_left_counter > 0) {
        cv_signal(el, intersection_lock);
    }
    if (n_counter > 0 && s_counter > 0 && e_right_counter > 0 && w_left_counter > 0 && s_left_counter > 0 && e_left_counter > 0) {
        cv_signal(e, intersection_lock);
    }
    lock_release(intersection_lock);
}
