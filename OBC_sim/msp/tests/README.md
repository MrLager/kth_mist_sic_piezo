# MSP library tests
Contains test cases for both the experiment side and OBC side of the MSP
library. Use `make test` to run all the test cases. Use `make clean`
after to remove all the generated binaries.

Some test cases need to be executed on a system with the `long` type in C
being exactly 4 bytes in size. Execute these tests with `make test32`. These
tests will fail unless the `long` size criterion is met. They also take more
time than usual to run as they test transmitting 4GB of data.

