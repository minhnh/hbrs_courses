#ifndef _ASSIGNMENT10_STATE_H_
#define _ASSIGNMENT10_STATE_H_
#include <vector>

#define EMPTY	0
#define X       1
#define O		-1
#define EDGE 	2

class State 
{
    private:
		int size_x;
		int size_y;
		std::vector<int> map;
		int ultility;
		int minimax_value;
		
		int get_value_at(int x, int y);
		
    public:
		State(std::vector<int>, int x, int y);
		~State();
		void calculate_ultility();
		int get_ultility();
		int get_minimax();
		void set_minimax(int value);
		std::vector<int> get_map();
		void print_map();
};

#endif /* _ASSIGNMENT10_STATE_H_ */
