#ifndef AGENT_HPP
#define AGENT_HPP


class Agent
{
    public:
        Agent(std::string mapData);
        ~Agent();

        void run();
        int bfs();
        int dfs();
        void get_map_data(int,int,int,int,int);


    private:

        void dfs_re(int x, int y, int * max_depth_p, int * dust_num);
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
        int dfs_checked_node;
        int dfs_stored_node;
};



#endif
