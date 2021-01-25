// textentering.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include<string>

using namespace std;
int step = 0;
int main()
{
    /*
    ofstream enter("train.txt", ios::app);
    if (step == 0)
    {
        for (int i = 1; i < 669; i++)
        {
            string everyline = "data/obj/" + to_string(i) + ".jpg";
            enter << everyline << endl;
            cout << "writing to file " << endl;
            if (i == 668)step = 1;
        }
    }
    if (step == 1)
    {
        for (int i = 2000; i < 2668; i++)
        {
            string everyline = "data/obj/" + to_string(i) + ".jpg";
            enter << everyline << endl;
            cout << "writing to file " << endl;
            if (i == 2667)step = 2;
        }
    }
    if (step == 2)
    {
        for (int i = 3000; i < 3668; i++)
        {
            string everyline = "data/obj/" + to_string(i) + ".jpg";
            enter << everyline << endl;
            cout << "writing to file " << endl;
            if (i == 3667)
            {
                step = 3; 
            }
        }
    }
    if (step == 3)
    {
        for (int i = 1; i < 1998; i++)
        {
            string everyline = "data/obj/notobj_" + to_string(i) + ".jpg";
            enter << everyline << endl;
            cout << "writing to file " << endl;
            if (i == 1997)
            {
                step = 4;
            }
        }
    }
    enter.close();*/

    for (int i = 1; i < 1998; i++)
    {
        ofstream file;
        file.open("notobj_" + to_string(i) + ".txt");
        file.close();
    }

    return 0;

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
