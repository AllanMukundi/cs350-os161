#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>
#include <array.h>

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

int light;
int cars; // number of cars before handing over
int intersection_counter, n_counter, s_counter, w_counter, e_counter;
struct array *q;
struct lock *intersection_lock;
struct cv *n, *s, *w, *e;

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
    light = -1;
    cars = 20;
    intersection_counter = 0;
    n_counter = 0;
    s_counter = 0;
    w_counter = 0;
    e_counter = 0;

    q = array_create();
    
    intersection_lock = lock_create("Intersection Lock");

    n = cv_create("North");
    s = cv_create("South");
    w = cv_create("West");
    e = cv_create("East");

    KASSERT(q != NULL);
    KASSERT(intersection_lock != NULL);
    KASSERT(n != NULL);
    KASSERT(s != NULL);
    KASSERT(w != NULL);
    KASSERT(e != NULL);
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
    KASSERT(q != NULL);
    KASSERT(intersection_lock != NULL);
    KASSERT(n != NULL);
    KASSERT(s != NULL);
    KASSERT(w != NULL);
    KASSERT(e != NULL);

    array_destroy(q);
    lock_destroy(intersection_lock);
    cv_destroy(n);
    cv_destroy(s);
    cv_destroy(w);
    cv_destroy(e);
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
sleep_on_cv(Direction origin) {
    if (origin == north) {
        n_counter += 1;
        cv_wait(n, intersection_lock);
        n_counter -= 1;
    } else if (origin == south) {
        s_counter += 1;
        cv_wait(s, intersection_lock);
        s_counter -= 1;
    } else if (origin == west) {
        w_counter += 1;
        cv_wait(w, intersection_lock);
        w_counter -= 1;
    } else if (origin == east) {
        e_counter += 1;
        cv_wait(e, intersection_lock);
        e_counter -= 1;
    }
}

void 
place_in_queue(Direction origin) {
    for (unsigned int i = 0; i < array_num(q); ++i) {
        if (*(unsigned int *)array_get(q, i) == origin) {
            return;
        }
    }
    array_add(q, (void *)&origin, NULL);
}

void
intersection_before_entry(Direction origin, Direction destination) 
{
    KASSERT(q != NULL);
    KASSERT(intersection_lock != NULL);
    KASSERT(n != NULL);
    KASSERT(s != NULL);
    KASSERT(w != NULL);
    KASSERT(e != NULL);

    (void)destination;

    lock_acquire(intersection_lock);
    if (intersection_counter == 0 && (e_counter + n_counter + s_counter + w_counter <= 0)) {
        light = origin;
    }
    cars -= 1;
    if (light == (signed)origin) {
        if (cars == 0) {
            if (e_counter + n_counter + s_counter + w_counter > 0) {
                place_in_queue(origin);
                sleep_on_cv(origin);
            } else {
                cars = 20;
            }
        }
    } else {
        place_in_queue(origin);
        sleep_on_cv(origin);
    }
    intersection_counter += 1;
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
    KASSERT(q != NULL);
    KASSERT(intersection_lock != NULL);
    KASSERT(n != NULL);
    KASSERT(s != NULL);
    KASSERT(w != NULL);
    KASSERT(e != NULL);

    (void)origin;
    (void)destination;

    lock_acquire(intersection_lock);
    intersection_counter -= 1;
    if (intersection_counter == 0 && (e_counter + n_counter + s_counter + w_counter > 0)) {
        if (array_num(q) > 0) {
            light = *(int *)array_get(q, 0);
            array_remove(q, 0);
        }
        if (light == north) {
            cv_broadcast(n, intersection_lock);
        } else if (light == south) {
            cv_broadcast(s, intersection_lock);
        } else if (light == west) {
            cv_broadcast(w, intersection_lock);
        } else if (light == east) {
            cv_broadcast(e, intersection_lock);
        }
        cars = 20;
    }
    lock_release(intersection_lock);
}
