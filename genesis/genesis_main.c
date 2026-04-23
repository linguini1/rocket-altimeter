/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <nuttx/lib/builtin.h>

#include <errno.h>
#include <spawn.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: start_process
 *
 * Description:
 *   Starts a builtin application as a process.
 *
 * Input Parameters:
 *   name - Name of the application to start as a process
 *   argv - The command line arguments to the process. Leave NULL for none.
 *          Arguments must start with the name of the program and end with a
 *          NULL terminator.
 *   pid - A pointer to return the PID in if successful.
 *
 * Returned Value:
 *   0 on success, an error code otherwise.
 *
 ****************************************************************************/

static int start_process(const char *name, char **argv, int *pid)
{
  int index;
  int ret = 0;
  FAR const struct builtin_s *builtin;
  posix_spawnattr_t attr;
  posix_spawn_file_actions_t file_actions;
  struct sched_param sched;

  DEBUGASSERT(name != NULL);
  DEBUGASSERT(pid != NULL);

  /* Get index of the application */

  index = builtin_isavail(name);
  if (index < 0)
    {
      ret = ENOENT;
      syslog(LOG_ERR | LOG_USER, "No application '%s'\n", name);
      goto ret_early;
    }

  /* Get attributes for the application */

  builtin = builtin_for_index(index);
  if (builtin == NULL)
    {
      ret = ENOENT;
      syslog(LOG_ERR | LOG_USER, "No application '%s'\n", name);
      goto ret_early;
    }

  /* Initialize required attributes */

  ret = posix_spawnattr_init(&attr);
  if (ret != 0)
    {
      goto ret_early;
    }

  ret = posix_spawn_file_actions_init(&file_actions);
  if (ret != 0)
    {
      goto ret_destr_attr;
    }

  /* Set the correct task size and priority */

  sched.sched_priority = builtin->priority;
  ret = posix_spawnattr_setschedparam(&attr, &sched);
  if (ret != 0)
    {
      goto ret_destr_file;
    }

  ret = posix_spawnattr_setstacksize(&attr, builtin->stacksize);
  if (ret != 0)
    {
      goto ret_destr_file;
    }

    /* If robin robin scheduling is enabled, then set the scheduling policy
     * of the new task to SCHED_RR before it has a chance to run.
     */

#if CONFIG_RR_INTERVAL > 0
  ret = posix_spawnattr_setschedpolicy(&attr, SCHED_RR);
  if (ret != 0)
    {
      goto ret_destr_file;
    }

  ret = posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSCHEDPARAM |
                                            POSIX_SPAWN_SETSCHEDULER);
  if (ret != 0)
    {
      goto ret_destr_file;
    }

#else
  ret = posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSCHEDPARAM);
  if (ret != 0)
    {
      goto ret_destr_file;
    }

#endif

  /* Spawn process */

  *pid = task_spawn(builtin->name, builtin->main, &file_actions, &attr,
                    argv != NULL ? &argv[1] : NULL, NULL);
  if (*pid < 0)
    {
      syslog(LOG_ERR | LOG_USER, "Task spawn failed: %d\n", *pid);
      ret = -(*pid);
      goto ret_destr_file;
    }

ret_destr_file:
  posix_spawn_file_actions_destroy(&file_actions);
ret_destr_attr:
  posix_spawnattr_destroy(&attr);
ret_early:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int err;
  int pid;
  int ret = EXIT_SUCCESS;

  syslog(LOG_INFO | LOG_USER, "Rocket altimeter genesis...\n");

  /* Spawn data processing stuff */

#ifdef CONFIG_ROCKETALT_FAKE_BARO
  err = start_process("fake_baro", NULL, &pid);
  if (err != 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't start fake_baro process: %d\n",
             err);
      ret = EXIT_FAILURE;
    }
  else
    {
      syslog(LOG_INFO | LOG_USER, "fake_baro [PID %d]\n", pid);
    }
#endif

#ifdef CONFIG_ROCKETALT_PROCESSING
  err = start_process("altitude_fusion", NULL, &pid);
  if (err != 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Couldn't start altitude_fusion process: %d\n", err);
      ret = EXIT_FAILURE;
    }
  else
    {
      syslog(LOG_INFO | LOG_USER, "altitude_fusion [PID %d]\n", pid);
    }

  err = start_process("velocity_fusion", NULL, &pid);
  if (err != 0)
    {
      syslog(LOG_ERR | LOG_USER,
             "Couldn't start velocity_fusion process: %d\n", err);
      ret = EXIT_FAILURE;
    }
  else
    {
      syslog(LOG_INFO | LOG_USER, "velocity_fusion [PID %d]\n", pid);
    }

  err = start_process("events_topic", NULL, &pid);
  if (err != 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't start events_topic process: %d\n",
             err);
      ret = EXIT_FAILURE;
    }
  else
    {
      syslog(LOG_INFO | LOG_USER, "events_topic [PID %d]\n", pid);
    }
#endif

    /* Start deployment logic */

#ifdef CONFIG_ROCKETALT_DEPLOYMENT
  err = start_process("deployment", NULL, &pid);
  if (err != 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't start deployment process: %d\n",
             err);
      ret = EXIT_FAILURE;
    }
  else
    {
      syslog(LOG_INFO | LOG_USER, "deployment [PID %d]\n", pid);
    }
#endif

    /* Start the logging process */

#ifdef CONFIG_ROCKETALT_LOGGING
  err = start_process("logger", NULL, &pid);
  if (err != 0)
    {
      syslog(LOG_ERR | LOG_USER, "Couldn't start logger process: %d\n", err);
      ret = EXIT_FAILURE;
    }
  else
    {
      syslog(LOG_INFO | LOG_USER, "logger [PID %d]\n", pid);
    }
#endif

    /* Start NSH if this program was the entry-point */

#ifdef CONFIG_SYSTEM_NSH
  if (strcmp(CONFIG_INIT_ENTRYNAME, "nsh_main") != 0)
    {
      syslog(LOG_INFO | LOG_USER, "Conditions met to start NSH...\n");

      err = start_process("nsh", NULL, &pid);
      if (err != 0)
        {
          syslog(LOG_ERR | LOG_USER, "Couldn't start NSH: %d\n", err);
          ret = EXIT_FAILURE;
        }
      else
        {
          syslog(LOG_INFO | LOG_USER, "NSH [PID: %d]\n", pid);
        }
    }
#endif

  return ret;
}
