/**
 * Author: Dhruv Rawat
 */

#include <iostream>
#include <fstream>
#include <string>
#include <stdint.h>
#include <sstream>
#include <bitset>

using namespace std;

float hex_to_float_reverse(string hexadecimal)
{
    stringstream ss;
    ss << hex << hexadecimal;
    unsigned n;
    ss >> n;

    bitset<32> binary(n);

    uint32_t num = (int)(binary.to_ulong());

    float f;
    f = *((float *)&num); // Interpret integer as a float

    return f;
}

int16_t hex_to_int16_reverse(string hexadecimal)
{
    stringstream ss;
    ss << hex << hexadecimal;
    unsigned n;
    ss >> n;

    bitset<16> binary(n);

    int16_t num = (int16_t)(binary.to_ulong());

    return num;
}

int main(int argc, char *argv[])
{

    if (argc != 4 && argc != 6)
    {
        cout << "Incorrect number of commands" << endl;
        exit(1);
    }

    ifstream readFile{argv[2]};
    ofstream outFile{argv[3]};

    int tokensPerHex = 0;
    int indexOfLastTokenInHex = 0;

    // Size of packages we decode depend on the size of conversion
    if (*argv[1] == 'a')
    { // 32 bit conversion
        tokensPerHex = 4;
    }
    else if (*argv[1] == 'b' || *argv[1] == 'c')
    { // 16 bit conversion
        tokensPerHex = 2;
    }

    indexOfLastTokenInHex = tokensPerHex * 2 - 2;

    while (!readFile.eof())
    {
        string hex = "";

        // Make the hexedecmal string in Big Endian
        for (int i = 0; i < tokensPerHex; i++)
        {
            string val = "";
            readFile >> val;
            hex += val;
        }

        if (hex.length() == tokensPerHex * 2)
        {
            string toConvert = "";

            // Reverse endianess to Little Endian
            if (*argv[1] == 'a' || *argv[1] == 'b' || *argv[1] == 'c')
            {
                for (int i = indexOfLastTokenInHex; i >= 0; i = i - 2)
                {
                    toConvert += hex.substr(i, 2);
                }
            }

            if (*argv[1] == 'a')
            { // Convert hex to floating point and reverse endianess
                float result = hex_to_float_reverse(toConvert);
                outFile << result << endl;
            }
            else if (*argv[1] == 'b' || *argv[1] == 'c')
            { // Convert hex to uint16_t and reverse endianes
                int16_t result = hex_to_int16_reverse(toConvert);

                if (*argv[1] == 'c')
                {
                    float scaleFactor = atof(argv[4]);
                    float calibration = atof(argv[5]);
                    result = result / scaleFactor - calibration;
                }

                outFile << result << endl;
            }
        }
    }

    readFile.close();
    outFile.close();
}