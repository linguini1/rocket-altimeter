/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../common/config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TODO: decide correct buffer sizes for everything */

#define LINE_BUFLEN (32)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: expect_newline
 *
 * Description:
 *   Parse the next line of the file, expecting it to be a new line.
 *
 * Input Parameters:
 *   file - The file to parse.
 *
 * Returned Value:
 *   0 on success, error code on failure. EINVAL when there was not a newline.
 *
 ****************************************************************************/

static int expect_newline(FILE *file)
{
  char newline_buf[2];

  if (fgets(newline_buf, sizeof(newline_buf), file) == NULL)
    {
      return errno;
    }

  if (newline_buf[0] != '\n')
    {
      return EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: expect_apogee
 *
 * Description:
 *   Parse the configuration file, expecting the apogee entry next.
 *
 * Input Parameters:
 *   file - The file to parse from
 *   apogee - Where to store the apogee value.
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 *   EINVAL: The entry format was incorrect.
 *
 ****************************************************************************/

static int expect_apogee(FILE *file, float *apogee)
{
  char buf[LINE_BUFLEN];
  char *cur;

  if (fgets(buf, sizeof(buf), file) == NULL)
    {
      return errno;
    }

  /* Make sure the string contains the apogee designator, and that the apogee
   * designator is also the start of the line.
   */

  cur = strstr(buf, "apogee=");
  if (cur == NULL || cur != buf)
    {
      return EINVAL;
    }

  /* Start parsing after the apogee designator */

  cur += sizeof("apogee=") - 1;

  errno = 0;
  *apogee = strtof(cur, &cur);
  if (errno != 0)
    {
      return errno; /* This checks for ERANGE */
    }

  /* Ensure we ended float parsing at the newline character */

  if (*cur != '\n') return EINVAL;

  return 0;
}

/****************************************************************************
 * Name: expect_channel_header
 *
 * Description:
 *   Parses the next line in the file expecting it to be a channel header.
 *
 * Input Parameters:
 *   file - The file to parse from
 *   channo - Where to store the parsed channel number
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 ****************************************************************************/

static int expect_channel_header(FILE *file, int *channo)
{
  char buf[LINE_BUFLEN];
  char *cur;

  /* First, try to read the channel header */

  if (fgets(buf, sizeof(buf), file) == NULL)
    {
      return errno;
    }

  /* Make sure the string contains the channel designator, and that the
   * channel designator is also the start of the line.
   */

  cur = strstr(buf, "[chan ");
  if (cur == NULL || cur != buf) return EINVAL;

  /* Start parsing after the channel designator */

  cur += sizeof("[chan ") - 1;

  errno = 0;
  *channo = strtol(cur, &cur, 10);
  if (errno) return errno; /* Catches ERANGE */

  /* Ensure that the channel number is followed by the closing brace and EOL
   */

  if (strstr(cur, "]\n") != cur) return EINVAL; /* Bad format */

  return 0;
}

/****************************************************************************
 * Name: parse_condition_type
 *
 * Description:
 *   Parse a type string into a condition enum.
 *
 *   NOTE: The value stored in `str` will contain the address in the string
 *   where parsing stopped (the character right after the type).
 *
 * Input Parameters:
 *   str - A reference to the pointer of the string to parse
 *   cond - The location to store the condition
 *
 * Returned Value:
 *   0 on success, EINVAL on unrecognized type.
 *
 ****************************************************************************/

static int parse_condition_type(char **str, int *cond)
{
  if (!strncmp("alt", *str, sizeof("alt") - 1))
    {
      *cond = COND_ALT;
      *str += sizeof("alt") - 1;
      return 0;
    }
  else if (!strncmp("apogee", *str, sizeof("apogee") - 1))
    {
      *cond = COND_APOGEE;
      *str += sizeof("apogee") - 1;
      return 0;
    }
  else if (!strncmp("time", *str, sizeof("time") - 1))
    {
      *cond = COND_TIME;
      *str += sizeof("time") - 1;
      return 0;
    }

  return EINVAL;
}

/****************************************************************************
 * Name: expect_channel_condition
 *
 * Description:
 *   Parses the next line in the file expecting it to be a channel condition.
 *
 *   WARNING: The parsed condition flag is OR'd into `chan->conditions`, so it
 *   must be initialized before this call to 0 or the previously parsed
 *   conditions.
 *
 * Input Parameters:
 *   file - The file to parse
 *   chan - The channel configuration to store the results in
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 *   ENOENT: No more channel conditions to parse.
 *
 ****************************************************************************/

static int expect_channel_condition(FILE *file, struct chan_config_s *chan)
{
  char buf[LINE_BUFLEN];
  char *cur;
  int condition = 0;
  int err;

  /* First, try to read the condition line */

  if (fgets(buf, sizeof(buf), file) == NULL)
    {
      if (feof(file)) return ENOENT; /* Out of conditions to parse */
      return errno;                  /* Otherwise it's an error */
    }

  /* If we've hit a new line character, then we're out of conditions */

  if (buf[0] == '\n') return ENOENT;

  /* Make sure the string contains the condition designator, and that the
   * condition designator is also the start of the line.
   */

  cur = strstr(buf, "condition=");
  if (cur == NULL || cur != buf) return EINVAL;

  /* Start parsing after the condition designator */

  cur += sizeof("condition=") - 1;

  /* Parse the condition type */

  err = parse_condition_type(&cur, &condition);
  if (err) return err;

  /* Check if the condition type needs an argument parsed */

  switch (condition)
    {
    case COND_ALT:
      if (*cur++ != ',') return EINVAL; /* Advance the trailing comma */
      errno = 0;
      chan->altitude = strtof(cur, &cur);
      if (errno) return errno; /* Catches ERANGE */
      break;

    case COND_TIME:
      if (*cur++ != ',') return EINVAL; /* Advance the trailing comma */
      errno = 0;
      chan->time = strtoul(cur, &cur, 10);
      if (errno) return errno; /* Catches ERANGE/EINVAL */
      break;

    case COND_APOGEE:
      break; /* No parsing necessary */
    }

  chan->conditions |= condition; /* Store the parsed condition */

  /* Ensure that we're at the end of the parsed line. */

  if (*cur != '\n') return EINVAL;

  return 0;
}

/****************************************************************************
 * Name: expect_channel
 *
 * Description:
 *   Parses a channel entry in the file, expected to start at the next file
 *   read.
 *
 *   NOTE: This function will store the channel in the configurations `chans`
 *   member at index `id - 1`, where `id` is the ID assigned to the channel in
 *   the file.
 *
 *   WARN: This function expects any uninitialized channel in `chans` to have
 *   a `conditions` member equal to 0, since this is an invalid configuration
 *   for any channel (and can therefore be used to detect uninitialized
 *   slots). This function will only increment `config->nchans` if the parsed
 *   channel is getting stored in an uninitialized slot. Overwriting a slot
 *   does not change the number of channels stored.
 *
 *   WARN: This function expects `config->nchans` to be initialized to the
 *   correct value before calling, since the value is incremented on
 *   successful storage.
 *
 * Input Parameters:
 *   file - The file to parse the channel entry from
 *   config - The configuration structure to store the result into
 *
 * Returned Value:
 *   0 on success, error code on failure.
 *
 *   ENOENT: There were no more channels remaining in the file
 *   ENFILE: The channel ID was invalid for the capacity of `config->chans`
 *   EINVAL: A condition or its argument was invalid
 *
 ****************************************************************************/

static int expect_channel(FILE *file, struct config_s *config)
{
  int channo = -1;
  int err;

  /* Get the channel number */

  err = expect_channel_header(file, &channo);
  if (feof(file)) return ENOENT; /* Hit EOF, no more channels */
  if (err) return err;           /* Otherwise it's a regular error */

  /* Check if the channel number is valid */

  if (channo <= 0 || channo > CONF_MAX_DEPCHANS)
    {
      return ENFILE;
    }

  /* If the channel number is valid, parse the conditions until we run out.
   *
   * We also increment the number of channels if we're not overwriting a
   * filled slot.
   */

  if (config->chans[channo - 1].conditions == 0) config->nchans++;

  while (err != ENOENT)
    {
      err = expect_channel_condition(file, &config->chans[channo - 1]);
      if (err && err != ENOENT) return err;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rocketalt_config_from_file
 *
 * Description:
 *   Read in the contents of the file and return a configuration structure.
 *
 * Input Parameters:
 *   path - The file to read the configuration from
 *   config - The config struct to populate with file data
 *
 * Returned Value:
 *   0 on success, an error code on failure.
 *
 ****************************************************************************/

int rocketalt_config_from_file(const char *path, struct config_s *config)
{
  int err = 0;
  FILE *file;

  file = fopen(path, "r");
  if (file == NULL)
    {
      return errno;
    }

  /* Keep reading lines in the file. At each line, decide what we should
   * expect to read so that we can throw errors if we reach something
   * unexpected.
   */

  /* The file must start with an apogee entry */

  err = expect_apogee(file, &config->apogee);
  if (err) goto early_ret;

  /* The apogee entry is followed by a newline */

  err = expect_newline(file);
  if (err) goto early_ret;

  /* Next, we have a variable number of channels.
   *
   * We first set the number of channels equal to zero. It will be incremented
   * as we parse.
   *
   * We continue in a loop parsing the channels. The signal that there are no
   * channels remaining is a return value of `ENOENT`.
   */

  memset(config->chans, 0, sizeof(config->chans));
  config->nchans = 0;

  while (err != ENOENT)
    {
      err = expect_channel(file, config);
      if (err && err != ENOENT)
        {
          goto early_ret; /* Some issue reading the file */
        }
    }

  if (err == ENOENT) err = 0;

early_ret:
  fclose(file);
  return err;
}

/****************************************************************************
 * Name: rocketalt_config_getchan
 *
 * Description:
 *   Get a channel by its ID.
 *
 * Input Parameters:
 *   config - The configuration to get the channel from
 *   chanid - The channel ID of the channel to get
 *
 * Returned Value:
 *   A pointer to the channel on success, NULL on failure.
 *
 ****************************************************************************/

const struct chan_config_s *
rocketalt_config_getchan(const struct config_s *config, uint8_t chanid)
{
  if (chanid > array_len(config->chans))
    {
      return NULL;
    }

  if (config->chans[chanid - 1].conditions == 0)
    {
      return NULL; /* Empty slot */
    }

  return &config->chans[chanid - 1];
}
