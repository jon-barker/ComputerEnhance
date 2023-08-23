#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

struct Instruction
{
    uint8_t opcode : 6;
    uint8_t D : 1;
    uint8_t W : 1;
    uint8_t MOD : 2;
    uint8_t REG : 3;
    uint8_t RM : 3;
};

void decode(const std::vector<char> *buffer)
{
    Instruction inst;
    inst.opcode = ((*buffer)[0] & 0b11111100) >> 2;
    inst.D = ((*buffer)[0] & 0b00000010) >> 1;
    inst.W = (*buffer)[0] & 0b00000001;
    inst.MOD = ((*buffer)[1] & 0b11000000) >> 6;
    inst.REG = ((*buffer)[1] & 0b00111000) >> 3;
    inst.RM = (*buffer)[1] & 0b00000111;

    if (static_cast<int>(inst.opcode) == 34)
    {
        std::cout << "mov ";
    };

    char src;
    char dst;
    if (static_cast<int>(inst.D) == 1)
    {
        src = inst.RM;
        dst = inst.REG;
    }
    else
    {
        src = inst.REG;
        dst = inst.RM;
    }

    if (inst.W == 1)
    {
        switch (static_cast<int>(dst))
        {
        case 0:
            std::cout << "ax";
            break;
        case 1:
            std::cout << "cx";
            break;
        case 2:
            std::cout << "dx";
            break;
        case 3:
            std::cout << "bx";
            break;
        case 4:
            std::cout << "sp";
            break;
        case 5:
            std::cout << "bp";
            break;
        case 6:
            std::cout << "si";
            break;
        case 7:
            std::cout << "di";
            break;
        }
    }
    else
    {
        switch (static_cast<int>(dst))
        {
        case 0:
            std::cout << "al";
            break;
        case 1:
            std::cout << "cl";
            break;
        case 2:
            std::cout << "dl";
            break;
        case 3:
            std::cout << "bl";
            break;
        case 4:
            std::cout << "ah";
            break;
        case 5:
            std::cout << "ch";
            break;
        case 6:
            std::cout << "dh";
            break;
        case 7:
            std::cout << "bh";
            break;
        }
    }

    std::cout << ", ";

    if (inst.W == 1)
    {
        switch (static_cast<int>(src))
        {
        case 0:
            std::cout << "ax";
            break;
        case 1:
            std::cout << "cx";
            break;
        case 2:
            std::cout << "dx";
            break;
        case 3:
            std::cout << "bx";
            break;
        case 4:
            std::cout << "sp";
            break;
        case 5:
            std::cout << "bp";
            break;
        case 6:
            std::cout << "si";
            break;
        case 7:
            std::cout << "di";
            break;
        }
    }
    else
    {
        switch (static_cast<int>(src))
        {
        case 0:
            std::cout << "al";
            break;
        case 1:
            std::cout << "cl";
            break;
        case 2:
            std::cout << "dl";
            break;
        case 3:
            std::cout << "bl";
            break;
        case 4:
            std::cout << "ah";
            break;
        case 5:
            std::cout << "ch";
            break;
        case 6:
            std::cout << "dh";
            break;
        case 7:
            std::cout << "bh";
            break;
        }
    }

    std::cout << '\n';
};

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <file_path>" << std::endl;
        return 1;
    }

    std::ifstream file(argv[1], std::ios::binary);

    if (!file)
    {
        std::cerr << "Error opening file: " << argv[1] << std::endl;
        return 1;
    }

    // Find the size of the file
    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // Read the file into a vector
    std::vector<char> buffer(fileSize);
    file.read(buffer.data(), fileSize);

    file.close();

    std::cout << "bits 16\n";
    std::cout << '\n';
    for (int i = 0; i < buffer.size(); i = i + 2)
    {
        std::vector<char> instructionBuffer(buffer.begin() + i, buffer.begin() + i + 2);
        decode(&instructionBuffer);
    }

    return 0;
}