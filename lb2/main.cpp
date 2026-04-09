#include <iostream>
#include "modules/DotProduct/DotProduct.hpp"

int main() {
    std::vector<double> a = {1e100, 1.0, -1e100};
    std::vector<double> b = {1.0, 1.0, 1.0};

    double result = DotProduct::compute(a, b);

    double wrong = 0;
    for (size_t i = 0; i < 3; i++) {
        wrong += a[i] * b[i];
    }

    std::cout<<"Прямое вычисление: "<<wrong<<std::endl;
    std::cout<<"Скалярное произведение: "<<result<<std::endl;
    
    return 0;
}