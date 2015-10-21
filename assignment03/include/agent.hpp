#ifndef AGENT_HPP
#define AGENT_HPP


class Agent
{
    public:
        Agent();
        ~Agent();

        void run();
        int bfs();
        int dfs();
        
    private:
        
        void print_map();
        
};



#endif
