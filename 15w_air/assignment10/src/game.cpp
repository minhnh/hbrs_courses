#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <climits>
#include "game.hpp"
#include "state.hpp"

using namespace std;

Game::Game(int x, int y, int number_of_human_player, int mode)
{
    size_x = x;
    size_y = y;
    is_finished = false;

    if (mode < 1 || mode > 2) {
        cout << "Invalid mode. Exiting..." << endl;
        exit(1);
    }

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
        decide_move(player1, X, current_map);
        print_map(current_map);
        State a = State(current_map, size_x, size_y);
        a.calculate_ultility();
        cout << a.get_ultility() << endl;

        decide_move(player2, O, current_map);
        print_map(current_map);
		State b = State(current_map, size_x, size_y);
        b.calculate_ultility();
        cout << b.get_ultility() << endl;
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
    State state(current_map, size_x, size_y);
    state.get_ultility();
    if (state.is_terminal_state()) {
        is_finished = true;
        return;
    }
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
    state.get_ultility();
    if (state.is_terminal_state()) {
        is_finished = true;
        return;
    }
    if (mode == MINIMAX) {
        Operator op = decision_minimax(state, symbol);
        set_value_at(op.x, op.y, symbol, map);
    }
    else if (mode == ALPHA_BETA) {
        Operator op = decision_alpha_beta(state, symbol);
        set_value_at(op.x, op.y, symbol, map);
        cout << op.x << endl;
        cout << op.y << endl;
    }
}

Game::Operator Game::decision_alpha_beta(State & state, int symbol)
{
    vector<Operator> operators;
    for (int x = 0; x < size_x; x++) {
        for (int y = 0; y < size_y; y++) {
            if (state.get_value_at(x, y) == EMPTY) {
                int depth = 0;
                int alpha = INT_MAX;
                int beta = INT_MIN;
                Operator op;
                op.x = x;
                op.y = y;
                state.set_value_at(x, y, symbol);
                op.utility = value_alpha_beta(state, symbol, alpha, beta);
                state.set_value_at(x, y, EMPTY);
                operators.push_back(op);
            }
        }
    }
    int max_utility = -1;
    Operator max_op;
    for (vector<Operator>::iterator it = operators.begin() ; it != operators.end(); ++it) {
        if (abs(it->utility) > max_utility) {
            max_op = *it;
            max_utility = abs(it->utility);
        }
    }
    return max_op;
}

int Game::value_alpha_beta(State & state, int symbol, int & alpha, int & beta)
{
    // terminal check
    if (state.is_terminal_state())
        return state.get_ultility();

    if (symbol == X) {  // MAX turn
        // generate successor states
        int v = INT_MIN;
        for (int i = 0; i < size_x; i++) {
            for (int j = 0; j < size_y; j++) {
                if (state.get_value_at(i, j) == EMPTY) {
                    // execute move
                    state.set_value_at(i, j, X);
                    // recursive call
                    int result = value_alpha_beta(state, O, alpha, beta);
                    // clear move
                    state.set_value_at(i, j, EMPTY);
                    // pruning
                    v = (v > result) ? v : result;
                    if (v >= beta)
                        return v;
                    alpha = (alpha > v) ? alpha : v;
                }
            }
        }
    }
    else {              // MIN turn
        // generate successor states
        int v = INT_MAX;
        for (int i = 0; i < size_x; i++) {
            for (int j = 0; j < size_y; j++) {
                if (state.get_value_at(i, j) == EMPTY) {
                    // execute move
                    state.set_value_at(i, j, O);
                    // recursive call
                    int result = value_alpha_beta(state, X, alpha, beta);
                    // clear move
                    state.set_value_at(i, j, EMPTY);
                    // pruning
                    v = (v < result) ? v : result;
                    if (v <= alpha)
                        return v;
                    beta = (beta < v) ? beta : v;
                }
            }
        }
    }
}

Game::Operator Game::decision_minimax(State & state, int symbol)
{
    vector<Operator> operators;
    for (int x = 0; x < size_x; x++) {
        for (int y = 0; y < size_y; y++) {
            if (state.get_value_at(x, y) == EMPTY) {
                int depth = 0;
                Operator op;
                op.x = x;
                op.y = y;
                state.set_value_at(x, y, symbol);
                op.utility = value_minimax(state, symbol, depth);
                state.set_value_at(x, y, EMPTY);
                operators.push_back(op);
            }
        }
    }
    int max_utility = -1;
    Operator max_op;
    for (vector<Operator>::iterator it = operators.begin() ; it != operators.end(); ++it) {
        if (abs(it->utility) > max_utility) {
            max_op = *it;
            max_utility = abs(it->utility);
        }
    }
    return max_op;
}

int Game::value_minimax(State & state, int symbol, int & cur_depth)
{
    // terminal check
    if (state.is_terminal_state())
        return state.get_ultility();
    // depth limit
    cur_depth++;
    if (cur_depth > MAX_DEPTH)
        return state.get_ultility();

    if (symbol == X) {  // MAX turn
        // generate successor states
        vector<int> utilities;
        for (int i = 0; i < size_x; i++) {
            for (int j = 0; j < size_y; j++) {
                if (state.get_value_at(i, j) == EMPTY) {
                    // execute move
                    state.set_value_at(i, j, X);
                    // recursive call
                    utilities.push_back(value_minimax(state, O, cur_depth));
                    // clear move
                    state.set_value_at(i, j, EMPTY);
                }
            }
        }
        // Return max of the utilities collected
        cur_depth--;
        return *(max_element(utilities.begin(), utilities.end()));
    }
    else {              // MIN turn
        // generate successor states
        vector<int> utilities;
        for (int i = 0; i < size_x; i++) {
            for (int j = 0; j < size_y; j++) {
                if (state.get_value_at(i, j) == EMPTY) {
                    // execute move
                    state.set_value_at(i, j, O);
                    // recursive call
                    utilities.push_back(value_minimax(state, X, cur_depth));
                    // clear move
                    state.set_value_at(i, j, EMPTY);
                }
            }
        }
        // Return max of the utilities collected
        cur_depth--;
        return *(min_element(utilities.begin(), utilities.end()));
    }
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
