#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

/**
 * Log levels
 * INFO: Informational messages
 * WARNING: Warning messages
 * ERROR: Error messages
 *
 * To configure the log level, change the value of the `logLevel` variable
 */
enum LogLevel
{
  INFO,
  WARNING,
  ERROR
};

/**
 * Log a message with a timestamp and log level
 * @param level The log level
 * @param message The message to log
 * @return void
 *
 * To configure the log level, change the value of the `logLevel` variable
 */
void logMessage(LogLevel level, const String &message)
{
  const char *levelStr;
  switch (level)
  {
  case INFO:
    levelStr = "INFO";
    break;
  case WARNING:
    levelStr = "WARNING";
    break;
  case ERROR:
    levelStr = "ERROR";
    break;
  }
  Serial.print("[");
  Serial.print(millis());
  Serial.print("] ");
  Serial.print(levelStr);
  Serial.print(": ");
  Serial.println(message);
}

#endif
