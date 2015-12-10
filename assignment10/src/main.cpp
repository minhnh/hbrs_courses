#include <iostream>
#include "game.hpp"

int main(int argc, char* argv[]) {
    // Argument check
    int number_of_human = 2;
    switch (argc) {
        case 1:
            std::cout << "Using default file and order option" << std::endl << std::endl;
            new Game(6,7,number_of_human);
            break;
        case 2:
            number_of_human = std::stoi(argv[1]);
            new Game(6,7,number_of_human);
            break;
        default:
            std::cout << "Too many arguments" << std::endl << std::endl;
            return 2;
    }
    return 0;
}
