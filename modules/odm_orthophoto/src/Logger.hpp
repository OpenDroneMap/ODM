#pragma once

// STL
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

/*!
 * \brief   The Logger class is used to store program messages in a log file.
 * \details By using the << operator while printInCout is set, the class writes both to
 *          cout and to file, if the flag is not set, output is written to file only.
 */
class Logger
{
public:
    /*!
     * \brief Logger        Contains functionality for printing and displaying log information.
     * \param printInCout   Flag toggling if operator << also writes to cout.
     */
    Logger(bool isPrintingInCout = true);

    /*!
     *  \brief Destructor.
     */
    ~Logger();

    /*!
     * \brief print     Prints the contents of the log to file.
     * \param filePath  Path specifying where to write the log.
     */
    void print(std::string filePath);

    /*!
     * \brief isPrintingInCout  Check if console printing flag is set.
     * \return                  Console printing flag.
     */
    bool isPrintingInCout() const;

    /*!
     * \brief setIsPrintingInCout   Set console printing flag.
     * \param isPrintingInCout      Value, if true, messages added to the log are also printed in cout.
     */
    void setIsPrintingInCout(bool isPrintingInCout);

    /*!
     *  Operator for printing messages to log and in the standard output stream if desired.
     */
    template<class T>
    friend Logger& operator<< (Logger &log, T t)
    {
        // If console printing is enabled.
        if (log.isPrintingInCout_)
        {
            std::cout << t;
            std::cout.flush();
        }
        // Write to log.
        log.logStream_ << t;

        return log;
    }

private:
    bool isPrintingInCout_;         /*!< If flag is set, log is printed in cout and written to the log. */

    std::stringstream logStream_;   /*!< Stream for storing the log. */
};
