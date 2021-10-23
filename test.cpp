#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <dart/dart.hpp>
#include "SimCharacter.h"

int main()
{
    SimCharacter character("character-definition.json");
    return 0;
}
