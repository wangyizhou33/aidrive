#Build test

```
g++ tests/TestSuite.cpp -o TestSuite
```

#Prettify
```
clang-format -style=Microsoft -i *.cpp *.h  cost/*.h tests/*.h tests/*.cpp
```

#Build main
```
g++ main.cpp -o main
```