#include <iostream>
#include <limits>
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

    std::cout<<"Проверка long double\n";
    std::cout<<"digits10: "<<std::numeric_limits<long double>::digits10<<std::endl;
    std::cout<<"sizeof(long double): "<< sizeof(long double)<<" bytes"<<std::endl;
    std::cout<<"sizeof(double): "<<sizeof(double)<<" bytes"<<std::endl;
    
    return 0;
}