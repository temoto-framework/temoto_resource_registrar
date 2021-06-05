# RR Core library

The base resource management library used inside TeMoto.

This library was developed as part of a Masters thesis that can be found here:
[Design and Implementation of a Generalized Resource Management Architecture in the TeMoto Software Framework](https://www.ims.ut.ee/www-public2/at/2021/msc/atprog-courses-magistrit55-loti.05.036-allan-kustavus-text-20210520.pdf)

## How to build

Without tests:
```
mkdir build; cd build; cmake ..; sudo make install
```

With tests:

```
mkdir build; cd build; cmake .. -Dtest=ON; sudo make install; ./rr_core_test
```

## How to use
The best way to learn how to use the library is to read examples from the autest files that can be found [HERE](rr_core/test/rr_test.cpp). All base functionalities are covered in tests named accordingly. They can be seen as example implementations and can also be used as a template to extend the libraries functionalities.
