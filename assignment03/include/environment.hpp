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
        void print_map();

    private:
        
};

#endif
