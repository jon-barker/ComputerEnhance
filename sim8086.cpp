#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

constexpr int INSTRUCTION_SIZE = 2;
constexpr int MOV_OPCODE = 34;

struct Instruction
{
    uint8_t opcode : 6;
    uint8_t D : 1;
    uint8_t W : 1;
    uint8_t MOD : 2;
    uint8_t REG : 3;
    uint8_t RM : 3;
};

std::string registerToString(int value, bool wide)
{
    static const char *registers[] = {"al", "cl", "dl", "bl", "ah", "ch", "dh", "bh"};
    static const char *wideRegisters[] = {"ax", "cx", "dx", "bx", "sp", "bp", "si", "di"};
    return wide ? wideRegisters[value] : registers[value];
}

void decode(const std::vector<char> *buffer)
{
    if (buffer->size() < INSTRUCTION_SIZE)
    {
        std::cerr << "Invalid instruction size" << '\n';
        return;
    }

    Instruction inst;
    inst.opcode = ((*buffer)[0] & 0b11111100) >> 2;
    inst.D = ((*buffer)[0] & 0b00000010) >> 1;
    inst.W = (*buffer)[0] & 0b00000001;
    inst.MOD = ((*buffer)[1] & 0b11000000) >> 6;
    inst.REG = ((*buffer)[1] & 0b00111000) >> 3;
    inst.RM = (*buffer)[1] & 0b00000111;

    if (static_cast<int>(inst.opcode) == MOV_OPCODE)
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

    std::cout << registerToString(dst, inst.W) << ", " << registerToString(src, inst.W) << '\n';
};

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <file_path>" << '\n';
        return 1;
    }

    std::ifstream file(argv[1], std::ios::binary);

    if (!file)
    {
        std::cerr << "Error opening file: " << argv[1] << '\n';
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

    std::cout << "bits 16" << '\n';
    std::cout << '\n';
    for (int i = 0; i < buffer.size(); i = i + INSTRUCTION_SIZE)
    {
        std::vector<char> instructionBuffer(buffer.begin() + i, buffer.begin() + i + INSTRUCTION_SIZE);
        decode(&instructionBuffer);
    }

    return 0;
}