#include <stdlib.h>
#include <iostream>
#include "game.hpp"

using namespace std;

Game::Game(int x, int y, int number_of_human_player)
{
    size_x = x;
    size_y = y;
    is_finished = false;
    for (int i = 0; i < x*y; i++)
    {
        current_map.push_back(EMPTY);
    }

    if (number_of_human_player == 2)
    {
        player1 = true;
        player2 = true;
    }
    else if (number_of_human_player == 1)
    {
        player1 = true;
        player2 = false;
    }
    else
    {
        player1 = false;
        player2 = false;
    }

    print_map(current_map);

    while (!is_finished)
    {
        decide_move(player1,X,current_map);
        print_map(current_map);
        decide_move(player2,O,current_map);
        print_map(current_map);
    }
}

Game::~Game()
{

}

void Game::decide_move(bool is_human, int symbol, vector<int> &map)
{
    if (is_human)
    {
        human_move(symbol, map);
    }
    else
    {
        computer_move(symbol, map);
    }
}

void Game::human_move(int symbol, vector<int> &map)
{
    int x = 0;
    int y = 0;
    cout << "Choose your move: X Y :";
    cin >> x >> y;
    while (get_value_at(x,y,map) != EMPTY)
    {
        cout << "\nPlease choose a valid move: X Y :";
        cin >> x >> y;
    }
    set_value_at(x,y,symbol,map);
}

//Alpha-beta should be here
void Game::computer_move(int symbol, vector<int> &map)
{

    State state(current_map, size_x, size_y);

}

void Game::decision_alpha_beta(State & state)
{

}

Game::Operator Game::decision_minimax(State & state)
{
    vector<Operator> operators;
    for (int x = 0; x < size_x; x++) {
        for (int y = 0; y < size_y; y++) {
            if (state.get_value_at(x, y) == EMPTY) {
                struct Operator op;
                op.x = x;
                op.y = y;
                op.utility = value_minimax(state, x, y);
                operators.push_back(op);
            }
        }
    }
    int max_utility = 0;
    Operator max_op;
    for (vector<Operator>::iterator it = operators.begin() ; it != operators.end(); ++it) {
        if (it->utility > max_utility)
            max_op = *it;
    }
}

int Game::value_minimax(State & state, int x, int y)
{
    if (state.is_terminal_state())
        return state.get_ultility();
}

int Game::set_value_at(int x, int y, int Symbol, vector<int> &map)
{
    if (x >= 0 && y >= 0 && x < size_x && y < size_y)
    {
        map[x + y*size_x] = Symbol;
    }
    return 0;
}

int Game::get_value_at(int x, int y, vector<int> map)
{
    if (x < 0 || y < 0 || x >= size_x || y >= size_y)
    {
        return EDGE;
    }
    return map[x + y*size_x];
}

void Game::print_map(vector<int> map)
{
    int symbol = 0;
    for (int i = 0; i < size_x; i++)
    {
        for (int j = 0; j < size_y; j++)
        {
            symbol = get_value_at(i,j,map);
            if (symbol == EMPTY)
            {
                cout << "| ";
            }
            else if (symbol == X)
            {
                cout << "|X";
            }
            else if (symbol == O)
            {
                cout << "|O";
            }
        }
        cout << "|\n";
    }
    cout << endl;
}
