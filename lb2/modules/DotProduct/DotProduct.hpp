#pragma once
#include "modules/ExactDouble/ExactDouble.hpp"

class DotProduct {
    private:
    static constexpr int EXP_MIN = -1022;
    static constexpr int EXP_MAX = 1023;
    static constexpr int TABLE_SIZE = 2 * (EXP_MAX - EXP_MIN + 1);
    static constexpr int BIAS = 1023;

    std::array<double, TABLE_SIZE> table;
    int current_min_exp;
    int current_max_exp;

    int table_index (int exponent, int genus) const;
    void insert (double x);
    static int get_exponent (double x);
    void add_product (double a, double b);
    double get_result ();

    public:
    DotProduct () {
        table.fill(0.0);
        current_min_exp = 10000;
        current_max_exp = -10000;
    }

    static double compute (const std::vector<double>& a, const std::vector<double>& b);
};