#pragma once
#include <cstdint>
#include <bit>
#include <array>
#include <vector>

class ExactDouble {
    private:
    static constexpr int MANT_BITS = 52;
    static constexpr int EXP_BITS = 11;
    static constexpr int TOTAL_MANT = 53;
    static constexpr int SPLIT_BITS = TOTAL_MANT / 2;

    double value;

    public:
    static uint64_t as_raw (double x);
    static double from_raw (uint64_t raw);
    static int genus (double x);    // младший бит мантиссы
    static double geneve (double x);    // обнуление младшего бита
    static double msb (double x);   // обнуление дробной части мантиссы
    static void split (double x, double& x1, double& x2);

    ExactDouble () : value(0.0) {}
    explicit ExactDouble (double value) : value(value) {}

    operator double () const { return value; }

    static std::pair<double, double> add (double a, double b);
    static std::vector<double> mul (double a, double b);
};