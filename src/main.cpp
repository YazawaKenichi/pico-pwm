#include "main.hpp"
#include "core.hpp"

#define HAMMER_R_PIN 2
#define HAMMER_L_PIN 3

int main()
{
    MainNode node;
    while (true)
    {
        node.spin();
    }
    return 0;
}


