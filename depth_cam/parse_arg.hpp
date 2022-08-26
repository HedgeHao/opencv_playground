#include <iostream>

class Config
{
public:
    int mode = 0; // 0: main, 1: calibration
    int calibration_rows = 9;
    int calibration_cols = 15;

    void toString()
    {
        printf("[Arguments] >>>>>>>>>>>>>>>>>>>>>\n"
               "mode=%d\n"
               "calibration_rows=%d\n"
               "calibration_cols=%d\n"
               "[Arguments] <<<<<<<<<<<<<<<<<<<<\n",
               mode, calibration_rows, calibration_cols);
    }
};

std::string getNextArg(int argc, char **argv, int target)
{
    return target >= argc ? "" : std::string(argv[target]);
}

void parseArgument(int argc, char **argv, Config *config)
{
    int index = 1;
    bool printHelp = false;
    for (; index < argc; index++)
    {
        std::string arg = argv[index];
        if (arg == "-c" || arg == "--calibration")
        {
            config->mode = 1;
        }
        else
        {
            printHelp = true;
        }
    }

    if (printHelp)
    {
        printf("[Help]\n");
    }

    config->toString();
}