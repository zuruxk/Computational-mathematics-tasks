#include "ExactDouble.hpp"

uint64_t ExactDouble::as_raw (double x) {
    return std::bit_cast<uint64_t>(x);
}

double ExactDouble::from_raw (uint64_t raw) {
    return std::bit_cast<double>(raw);
}

int ExactDouble::genus (double x) {
    return static_cast<int>(as_raw(x) & 1);
}

double ExactDouble::geneve (double x) {
    uint64_t raw = as_raw(x);
    raw &= ~1ULL;
    return from_raw(raw);
}

double ExactDouble::msb (double x) {
    uint64_t raw = as_raw(x);
    raw &= ~((1ULL << MANT_BITS) - 1);
    return from_raw(raw);
}

void ExactDouble::split (double x, double& x1, double& x2) {
    const double factor = (1LL << (TOTAL_MANT - SPLIT_BITS)) + 1;  // 2^27 + 1
    double t = factor * x;
    x1 = t - (t - x);   // ст 26
    x2 = x - x1;    // мл 27
}

std::pair<double, double> ExactDouble::add (double a, double b) {
    double s = a + b;   // округленная сумма
    double r = (a - s) + b;     // остаток
    return {s, r};
}

std::vector<double> ExactDouble::mul (double a, double b) {
    std::vector<double> parts;
    
    bool a_odd = (genus(a) == 1);
    bool b_odd = (genus(b) == 1);
    
    double a1, a2, b1, b2;
    
    if (a_odd && b_odd) {   // оба нечет: 5 произведений
        double msb_a = msb(a);
        double a_rest = a - msb_a;
        
        split(a_rest, a1, a2);
        split(b, b1, b2);
        
        parts.push_back(msb_a * b);
        parts.push_back(a1 * b1);
        parts.push_back(a1 * b2);
        parts.push_back(a2 * b1);
        parts.push_back(a2 * b2);
    }
    else {    // хоть одно чет: 4 произведения
        split(a, a1, a2);
        split(b, b1, b2);
        
        parts.push_back(a1 * b1);
        parts.push_back(a1 * b2);
        parts.push_back(a2 * b1);
        parts.push_back(a2 * b2);
    }
    
    return parts;
}