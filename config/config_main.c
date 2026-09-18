/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include "../common/config.h"

/****************************************************************************
 * Pre-processor Definitions
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
 * Name: print_channel
 *
 * Description:
 *   Prints a channel's configuration.
 *
 * Input Parameters:
 *   sink - Where to print the output
 *   config - The configuration to print from
 *   id - The ID of the channel to print
 *
 ****************************************************************************/

static int print_channel(FILE *sink, const struct config_s *config,
                         uint8_t id)
{
  int conditions;
  int cur;
  int bwrote;
  const struct chan_config_s *chan;

  if (id > CONF_MAX_DEPCHANS)
    {
      fprintf(stderr, "No channel with ID %u\n", id);
      return ENOENT;
    }

  chan = &config->chans[id - 1];

  bwrote = fprintf(sink, "[chan %u]\n", id);
  if (bwrote < 0)
    {
      return errno;
    }

  for (cur = 1, conditions = chan->conditions; conditions != 0; cur <<= 1)
    {

      if (conditions & cur)
        {
          conditions = conditions ^ cur; /* Flip matching bit to zero */
        }
      else
        {
          continue; /* Try next condition */
        }

      switch (cur)
        {
        case COND_ALT:
          bwrote = fprintf(sink, "condition=alt,%.2f\n", chan->altitude);
          break;
        case COND_APOGEE:
          bwrote = fprintf(sink, "condition=apogee\n");
          break;
        case COND_TIME:
          bwrote = fprintf(sink, "condition=time,%u\n", chan->time);
          break;
        default:
          break;
        }

      if (bwrote < 0)
        {
          return errno;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: print_apogee
 *
 * Description:
 *   Print the predicted apogee from the configuration.
 *
 * Input Parameters:
 *   sink - Where to print the result
 *   config - The configuration to print
 *
 ****************************************************************************/

static int print_apogee(FILE *sink, const struct config_s *config)
{
  int bwrote;

  bwrote = fprintf(sink, "apogee=%.2f\n", config->apogee);
  if (bwrote < 0)
    {
      return errno;
    }

  return 0;
}

/****************************************************************************
 * Name: print_config
 *
 * Description:
 *   Prints the entire configuration.
 *
 * Input Parameters:
 *   sink - Where to print the result
 *   config - The configuration to print
 *
 ****************************************************************************/

static int print_config(FILE *sink, const struct config_s *config)
{
  int err;

  err = print_apogee(sink, config);
  if (err) return err;

  /* New line before channels */

  if (fputc('\n', sink) == EOF) return ferror(sink);

  for (unsigned i = 0; i < config->nchans; i++)
    {
      /* If the channel is not initialized (i.e. empty slot), then skip
       * printing it.
       */

      if (config->chans[i].conditions == 0) continue;

      /* Print the channel */

      err = print_channel(sink, config, i + 1);
      if (err) return err;

      /* New line between each channel except last one */

      if (i + 1 != config->nchans)
        {
          if (fputc('\n', sink) == EOF) return ferror(sink);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: parse_condition_type
 *
 * Description:
 *   Parse a type string into a condition enum.
 *
 * Input Parameters:
 *   arg - The argument string to parse
 *   cond - Where to store the parse condition value
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int parse_condition_type(const char *arg, int *cond)
{
  if (!strcmp("alt", arg))
    {
      *cond = COND_ALT;
      return 0;
    }
  else if (!strcmp("apogee", arg))
    {
      *cond = COND_APOGEE;
      return 0;
    }
  else if (!strcmp("time", arg))
    {
      *cond = COND_TIME;
      return 0;
    }

  return EINVAL;
}

/****************************************************************************
 * Name: config_to_file
 *
 * Description:
 *   Write a configuration to a file.
 *
 * Input Parameters:
 *   path - The path of the file to write to
 *   config - The configuration to write to the file
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int config_to_file(const char *path, const struct config_s *config)
{
  int err = 0;
  FILE *file;

  file = fopen(path, "w");
  if (file == NULL)
    {
      fprintf(stderr, "Could not open '%s': %d\n", path, errno);
      return errno;
    }

  err = print_config(file, config);
  if (err)
    {
      fprintf(stderr, "Couldn't write config file: %d\n", err);
      return err;
    }

  fclose(file);
  return err;
}

/****************************************************************************
 * Name: cmd_apogee
 *
 * Description:
 *   Parse the command line arguments for the apogee command.
 *
 * Input Parameters:
 *   argc - Argument count
 *   argv - Argument array
 *   write - The bool whether to write or read
 *   config - The configuration
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int cmd_apogee(int argc, char **argv, bool write,
                      struct config_s *config)
{
  if (write)
    {
      optind++; /* Look for the apogee value */

      if (argc <= optind)
        {
          fprintf(stderr, "Expected apogee value.\n");
          return EINVAL;
        }

      /* Modify configuration */

      config->apogee = strtof(argv[optind], NULL);
    }

  /* Show the apogee */

  print_apogee(stdout, config);
  return 0;
}

/****************************************************************************
 * Name: cmd_channel
 *
 * Description:
 *   Parse the command line arguments for the channel command.
 *
 * Input Parameters:
 *   argc - Argument count
 *   argv - Argument array
 *   write - The bool whether to write or read
 *   config - The configuration
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int cmd_channel(int argc, char **argv, bool write,
                       struct config_s *config)
{
  int channo;
  int cond;

  /* Look for the channel number */

  optind++;
  if (argc <= optind)
    {
      fprintf(stderr, "Expected channel number.\n");
      return EINVAL;
    }

  channo = atoi(argv[optind]);
  optind++;

  if (write)
    {
      /* We will be parsing a list of conditions */

      config->chans[channo - 1].conditions = 0; /* Reset conditions */

      /* Expect at least one condition */

      if (argc <= optind)
        {
          fprintf(stderr, "Expected at least one condition for channel %d\n",
                  channo);
          return EINVAL;
        }

      while (argc > optind)
        {
          /* Get the condition type */

          if (parse_condition_type(argv[optind], &cond))
            {
              fprintf(stderr, "Invalid condition type '%s'\n", argv[optind]);
              return EXIT_FAILURE;
            }

          config->chans[channo - 1].conditions |= cond;
          optind++; /* Next argument */

          /* Handle missing arguments */

          switch (cond)
            {
            case COND_ALT:
            case COND_TIME: /* Arguments needed; fall-through deliberate */
              if (argc <= optind)
                {
                  fprintf(stderr, "Expected condition argument.\n");
                  return EINVAL;
                }
              break;

            default: /* No arguments needed */
              break;
            }

          /* If the condition has arguments, parse them too */

          switch (cond)
            {
            case COND_ALT:
              config->chans[channo - 1].altitude = strtof(argv[optind], NULL);
              optind++;
              break;

            case COND_TIME:
              config->chans[channo - 1].time =
                  strtoul(argv[optind], NULL, 10);
              optind++;
              break;

            default:
              break; /* No arguments */
            }
        }
    }

  /* Show the channel that was selected */

  print_channel(stdout, config, channo);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  int c;
  int err;
  bool write = false;
  const char *configpath = NULL;
  struct config_s config;

  /* Parse command line arguments */

  while ((c = getopt(argc, argv, ":w")) != -1)
    {
      switch (c)
        {
        case 'w':
          write = true;
          break;

        case ':':
          fprintf(stderr, "Option -%c requires an argument.\n", optopt);
          return EXIT_FAILURE;

        case '?':
          fprintf(stderr, "Unknown option '-%c'.\n", optopt);
          break; /* Don't exit, parse other options */

        default:
          fprintf(stderr, "Usage: config [-w] configpath [args]\n");
          return EXIT_FAILURE;
        }
    }

  /* Get the configuration file path */

  if (argc <= optind)
    {
      fprintf(stderr, "Expected config path.\n");
      return EXIT_FAILURE;
    }

  configpath = argv[optind];
  optind++;

  /* Read in the configuration file */

  err = rocketalt_config_from_file(configpath, &config);
  if (err)
    {
      fprintf(stderr, "Error reading file '%s': %d\n", configpath, err);
      return EXIT_FAILURE;
    }

  /* No optional positional arguments were passed. */

  if (argc <= optind)
    {
      if (!write)
        {
          /* If we're reading, show config */

          print_config(stdout, &config);
          return EXIT_SUCCESS;
        }

      /* Otherwise, write the configuration file again. */

      goto write_config;
    }

  /* If we're here, there are positional arguments to parse. */

  /* We only need to look at the first optional. Is it apogee or channel?
   * In write mode, we look for subsequent values.
   * In read mode, we only look for the field designator(s).
   */

  if (!strcmp("apogee", argv[optind]))
    {
      err = cmd_apogee(argc, argv, write, &config);
    }
  else if (!strcmp("chan", argv[optind]))
    {
      err = cmd_channel(argc, argv, write, &config);
    }
  else
    {
      /* We don't know what this is */

      fprintf(stderr, "Unexpected field '%s'\n", argv[optind]);
      return EXIT_FAILURE;
    }

  /* Catch any errors from argument parsing */

  if (err)
    {
      return EXIT_FAILURE;
    }

  /* If we were in write mode, write the new configuration file since we
   * passed the arguments with success.
   */

write_config:

  if (write)
    {
      err = config_to_file(configpath, &config);
      if (err)
        {
          fprintf(stderr,
                  "Couldn't write updated configuration to '%s': %d\n",
                  configpath, err);
          return EXIT_FAILURE;
        }
    }

  return EXIT_SUCCESS;
}
