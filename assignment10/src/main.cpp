#include <iostream>
#include "game.hpp"

int main(int argc, char* argv[]) {
    // Argument check
    int number_of_human = 2;
    int mode = 1;
    switch (argc) {
        case 1:
            std::cout << "2-human mode" << std::endl << std::endl;
            new Game(6,7,number_of_human, mode);
            break;
        case 2:
            std::cout << "1-human mode, minimax" << std::endl << std::endl;
            number_of_human = std::stoi(argv[1]);
            new Game(6, 7, number_of_human, mode);
            break;
        case 3:
            std::cout << "1-human mode";
            number_of_human = std::stoi(argv[1]);
            mode = std::stoi(argv[2]);
            if (mode == ALPHA_BETA) {
                std::cout <<  ", alpha-beta" << std::endl;
            }
            else if (mode == MINIMAX) {
                std::cout <<  ", minimax" << std::endl;
            }
            new Game(6, 7, number_of_human, mode);
            break;
        default:
            std::cout << "Too many arguments" << std::endl << std::endl;
            return 2;
    }
    return 0;
}
