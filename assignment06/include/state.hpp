#ifndef STATE_HPP
#define STATE_HPP

enum Heuristics
{
    MANHATTAN = 0,
    MISPLACED
};

class State
{
    public:
        State(int Tiles[], int size_x, int size_y, Heuristics heuristics);
        ~State();
        int map[9];
        int map_size_x;
        int map_size_y;

        int f;
        int h;
        int depth;
        int zero_index;
        bool can_move_up;
        bool can_move_down;
        bool can_move_left;
        bool can_move_right;
        int last_move;

        void check_expandability();
        //Manhattan distance
        int find_heuristics_1();
        //Misplaced tiles
        int find_heuristics_2();

        int get_value_at(int x, int y);
        int get_index(int x, int y);

        void print();
    private:
};
#endif
