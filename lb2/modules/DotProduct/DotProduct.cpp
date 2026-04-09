#include "DotProduct.hpp"

int DotProduct::table_index (int exponent, int genus) const {
    return 2 * (exponent + 1024) + genus;
}

void DotProduct::insert (double x) {
    if (x == 0.0) return;
    
    int exp = get_exponent(x);
    int gen = ExactDouble::genus(x);
    int idx = table_index(exp, gen);
    
    if (table[idx] != 0.0) {
        double sum = table[idx] + x;
        table[idx] = 0.0;
        insert(sum);
    } else {
        table[idx] = x;
        current_min_exp = std::min(current_min_exp, exp);
        current_max_exp = std::max(current_max_exp, exp);
    }
}

int DotProduct::get_exponent (double x) {
    if (x == 0.0) return -BIAS;
    uint64_t raw = ExactDouble::as_raw(x);
    int exp = (raw >> 52) & 0x7FF;
    return exp - BIAS;
}

void DotProduct::add_product (double a, double b) {
    std::vector<double> parts = ExactDouble::mul(a, b);
    for (double part : parts) {
        insert(part);
    }
}

double DotProduct::get_result() {
    if (current_min_exp > current_max_exp) return 0.0;
    
    double sum = 0.0;
    
    for (int exp = current_min_exp; exp <= current_max_exp; ++exp) {
        int even_idx = table_index(exp, 0);
        int odd_idx = table_index(exp, 1);
        
        double even_val = table[even_idx];
        double odd_val = table[odd_idx];
        
        if (odd_val != 0.0) {
            odd_val = ExactDouble::geneve(odd_val);
        }
        
        if (even_val != 0.0) sum = sum + even_val;
        if (odd_val != 0.0) sum = sum + odd_val;
    }
    
    return sum;
}

double DotProduct::compute (const std::vector<double>& a, const std::vector<double>& b) {
    DotProduct acc;
    for (size_t i = 0; i < a.size(); ++i) {
        acc.add_product(a[i], b[i]);
    }
    return acc.get_result();
}