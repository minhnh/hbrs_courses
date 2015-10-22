#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

enum CellType
{
    CELL_DIRT = 0,
    CELL_SPACE,
    CELL_START,
    CELL_OBSTACLE
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

    private:
        
};

#endif
