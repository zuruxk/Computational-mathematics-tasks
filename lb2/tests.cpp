#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>
#include "modules/DotProduct/DotProduct.hpp"

long double exact_dot(const std::vector<double>& a, const std::vector<double>& b) {
    long double sum = 0.0L;
    for (size_t i = 0; i < a.size(); ++i) {
        sum += (long double)a[i] * (long double)b[i];
    }
    return sum;
}

void check(const std::vector<double>& a, const std::vector<double>& b, double expected) {
    double result = DotProduct::compute(a, b);
    EXPECT_DOUBLE_EQ(result, expected);
}

TEST(DotProductTest, NormalValues) {
    std::vector<double> a = {1.1, 2.2, 3.3, 4.4, 5.5};
    std::vector<double> b = {6.6, 7.7, 8.8, 9.9, 10.10};
    check(a, b, 152.35);
}

TEST(DotProductTest, SmallValues) {
    std::vector<double> a = {1e-200, 1e-200};
    std::vector<double> b = {1.0, 1.0};
    check(a, b, 2e-200);
}

TEST(DotProductTest, LargeValues) {
    std::vector<double> a = {1e200, 1e200};
    std::vector<double> b = {1.0, 1.0};
    check(a, b, 2e200);
}

TEST(DotProductTest, Cancellation) {
    std::vector<double> a = {1e100, 1.23456789, -1e100};
    std::vector<double> b = {1.0, 1.0, 1.0};
    check(a, b, 1.23456789);
}

TEST(DotProductTest, Commutativity) {
    std::vector<double> a = {1.1, 2.2, 3.3, 4.4};
    std::vector<double> b = {4.4, 3.3, 2.2, 1.1};
    
    double r1 = DotProduct::compute(a, b);
    double r2 = DotProduct::compute(b, a);
    
    EXPECT_DOUBLE_EQ(r1, r2);
}

TEST(DotProductTest, DenormalNumbers) {
    double min_normal = std::numeric_limits<double>::min();      // 2.225e-308
    double min_denormal = std::numeric_limits<double>::denorm_min(); // 5e-324
    
    std::vector<double> a1 = {min_normal, min_normal};
    std::vector<double> b1 = {1.0, 1.0};
    double result1 = DotProduct::compute(a1, b1);
    EXPECT_DOUBLE_EQ(result1, 2.0 * min_normal);
    
    std::vector<double> a2 = {min_denormal, min_denormal};
    std::vector<double> b2 = {1.0, 1.0};
    double result2 = DotProduct::compute(a2, b2);
    
    EXPECT_DOUBLE_EQ(result2, 2.0 * min_denormal);
}

TEST(DotProductTest, LargeRandomVectors) {
    std::mt19937_64 rng(std::chrono::steady_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> dist(-1e100, 1e100);
    
    size_t N = 100000;
    
    std::vector<double> a(N);
    std::vector<double> b(N);
    
    for (size_t i = 0; i < N; ++i) {
        a[i] = dist(rng);
        b[i] = dist(rng);
    }
    
    long double exact = exact_dot(a, b);
    double expected = (double)exact;
    
    double result = DotProduct::compute(a, b);
    
    EXPECT_DOUBLE_EQ(result, expected);
}

TEST(DotProductTest, ExactZero) {
    std::vector<double> a = {
        1e100,
        1.0,
        -1e100,
        -1.0
    };
    std::vector<double> b = {
        1.0,
        1.0,
        1.0,
        1.0
    };
        
    double result = DotProduct::compute(a, b);
    
    EXPECT_DOUBLE_EQ(result, 0.0);
    
    double naive = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        naive += a[i] * b[i];
    }
    
    std::cout<<"Алгоритм: "<<result<<std::endl;
    std::cout<<"Обычное вычисление: "<<naive<< std::endl;
}

TEST(DotProductTest, SubnormalPreserved) {
    double subnormal = 1e-320;
    
    std::vector<double> a = {subnormal, 1e100, -1e100};
    std::vector<double> b = {1.0, 1.0, 1.0};
    
    double result = DotProduct::compute(a, b);
    
    EXPECT_DOUBLE_EQ(result, subnormal);
    
    double naive = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        naive += a[i] * b[i];
    }
    
    EXPECT_EQ(naive, 0.0);

    std::cout<<"Алгоритм: "<<result<<std::endl;
    std::cout<<"Обычное вычисление: "<<naive<<std::endl;
}

int main() {
    ::testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}