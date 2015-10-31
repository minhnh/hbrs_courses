#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

enum CellType
{
    CELL_DIRT = 0,
    CELL_SPACE = 1,
    CELL_START = 2,
    CELL_OBSTACLE = 3
};

class Environment
{
    public:

        Environment();
        void run();
        int load_map(int map_index);
        void initialize_map();
        char get_value_at(int x, int y);
        void print_map();
        std::string map;
        int map_Height;
        int map_Width;
        int start_X;
        int start_Y;
        int number_of_dust;

    private:

};

class MapCell
{
    public:
        MapCell(int X, int Y);
        ~MapCell();
        void set_x(int);
        void set_y(int);
        void set_xy(int,int);
        bool is_valid();
        int x;
        int y;
};

#endif
