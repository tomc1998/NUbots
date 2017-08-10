#include "TestClass.h"

TestClass::TestClass() {
    setVar(0);
}

void TestClass::setVar(int x) {
    var = x;
}

int TestClass::getVar() {
    return var;
}
