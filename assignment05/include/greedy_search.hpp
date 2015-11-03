#ifndef GREEDY_SEARCH_HPP
#define GREEDY_SEARCH_HPP

class Greedy_search
{
    public:
        Greedy_search();
        void run();
    private:

};

class State
{
	public:
		State(int Tiles[][]);
		int map[][];
		int map_size_x;
		int map_size_y;
		
		int h1;
		int h2;
		
		//Manhattan distance
        int find_heuristics_1();
        //Misplaced tiles
        int find_heuristics_2();
	private:
}
#endif
