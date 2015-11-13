#ifndef STATE_HPP
#define STATE_HPP

#include <climits>
enum Heuristics
{
    MANHATTAN = 0,
    MISPLACED
};

enum Direction
{
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3
};

class State
{
    public:
        State(int Tiles[], int size_x, int size_y, Heuristics heuristics);
        ~State();
        int map[9];
        int map_size_x;
        int map_size_y;
        bool can_move_up;
        bool can_move_down;
        bool can_move_left;
        bool can_move_right;

        int f;
        int h;
        int depth;
        int zero_index;
        Direction last_move;

        int get_value_at(int x, int y);
        int get_index(int x, int y);

        void print();

    private:
        void set_expandability();
        //Manhattan distance
        int find_heuristics_1();
        //Misplaced tiles
        int find_heuristics_2();
};
#endif
