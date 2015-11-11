#include "Logger.hpp"


Logger::Logger(bool isPrintingInCout) : isPrintingInCout_(isPrintingInCout)
{

}

Logger::~Logger()
{

}

void Logger::print(std::string filePath)
{
    std::ofstream file(filePath.c_str(), std::ios::binary);
    file << logStream_.str();
    file.close();
}

bool Logger::isPrintingInCout() const
{
    return isPrintingInCout_;
}

void Logger::setIsPrintingInCout(bool isPrintingInCout)
{
    isPrintingInCout_ = isPrintingInCout;
}
