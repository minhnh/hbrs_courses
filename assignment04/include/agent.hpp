#ifndef AGENT_HPP
#define AGENT_HPP


class Agent
{
    public:
        Agent(std::string mapData);
        ~Agent();

        void run();
        int ids();
        void get_map_data(int,int,int,int,int);


    private:

        MapCell ids_re(int x, int y, int max_depth, int * depth_reached, char target);
        void ids_clear_map(int x, int y);
        void print_map();
        std::string map;
        char get_value_at(int x, int y);
        char set_value_at(int x, int y, char value);
        int get_index_at(int x, int y);
        int map_Height;
        int map_Width;
        int start_X;
        int start_Y;
        int max_number_of_dust;
        int ids_checked_node;
        int ids_stored_node;
};



#endif
