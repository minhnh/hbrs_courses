#ifndef _ASSIGNMENT10_GAME_H_
#define _ASSIGNMENT10_GAME_H_
#include <vector>
#include "state.hpp"

#define MINIMAX     1
#define ALPHA_BETA  2
#define MAX_DEPTH   20

class Game
{
    private:
        struct Operator
        {
            int x = -1;
            int y = -1;
            int utility = 0;
        };
        int size_x;
        int size_y;
        int mode;
        bool is_finished;
        bool player1;
        bool player2;
        std::vector<int> current_map;

        void decide_move(bool Player, int symbol, std::vector<int> &map);
        void human_move(int symbol, std::vector<int> &map);
        void computer_move(int symbol, std::vector<int> &map);
        Operator decision_alpha_beta(State &, int);
        Operator decision_minimax(State &, int);
        int value_minimax(State &, int, int &);
        int value_alpha_beta(State &, int, int&, int&);

    public:
        Game(int x, int y, int number_of_human_player, int mode);
        ~Game();
        int set_value_at(int x, int y, int symbol, std::vector<int> &map);
        int get_value_at(int x, int y, std::vector<int> map);
        void print_map(std::vector<int> map);


};

#endif // _ASSIGNMENT10_GAME_H_
