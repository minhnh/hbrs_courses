#ifndef _ASSIGNMENT10_GAME_H_
#define _ASSIGNMENT10_GAME_H_
#include <vector>

class Game 
{
    private:
		int size_x;
		int size_y;
		bool is_finished;
		bool player1;
		bool player2;
		std::vector<int> current_map;
		void decide_move(bool Player, int symbol, std::vector<int> &map);
		void human_move(int symbol, std::vector<int> &map);
		void computer_move(int symbol, std::vector<int> &map);
		

    public:
		Game(int x, int y, int number_of_human_player);
		~Game();
		int set_value_at(int x, int y, int symbol, std::vector<int> &map);
		int get_value_at(int x, int y, std::vector<int> map);
		void print_map(std::vector<int> map);


};

#endif // _ASSIGNMENT10_GAME_H_
