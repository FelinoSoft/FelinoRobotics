/*
 *  mutexLib.c
 *  Contains functions for using semaphore variables and accessing stuff
 *  in mutual exclusion
 */

// Custom type of variable
typedef short TMutex;

/*
 *  Tries to acquire the mutex variable. If failes, retries giving other
 *  threads time to acquire it
 */
void AcquireMutex(TMutex &nMutex)
{
  while (true)
  {
    // Loop until mutex is obtained
   ++nMutex;
    if (nMutex == 1)
      return;  // Succeeded

    // Failed, try again
    --nMutex;
    wait1Msec(1); // To force timeslice to end and give other threads time
  }
}

/*
 *  Releases the mutex variable
 */
void ReleaseMutex(TMutex &nMutex)
{
  --nMutex;
}
