#include <iostream>
#include <stdio.h>

int main(int argc, char* argv[])
{

	int num1, num2;
	std::cin >> num1 >> num2;
	
	std::cout << "num1 + num2 : " << num1 + num2;
	std::cout << "\nnum1 - num2 : " << num1 - num2;
	std::cout << "\nnum1 * num2 : " << num1 * num2;
	std::cout << "\nnum1 / num2 : " << num1 / num2;
	
	return 0;
}
