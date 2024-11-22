#ifndef SYMPOLE_ALI_
#define SYMPOLE_ALI_

#include <iostream>
#include <string>
#include <array>


void strip(std::string &_s);
const std::string linksS[5] = {"66.5", "335.8", "183", "55", "70"};

typedef std::array<std::string, 5> AngS;
typedef std::array<std::string, 4> VecS;
typedef std::array<VecS, 4> MatS;

std::string prodTwoString(std::string &a, std::string &b);
std::string sumTwoString(std::string &a, std::string &b);

MatS operator*(const MatS &a, const MatS &b);

const MatS IDENS = {
    VecS({"1", "0", "0", "0"}),
    VecS({"0", "1", "0", "0"}),
    VecS({"0", "0", "1", "0"}),
    VecS({"0", "0", "0", "1"})};

MatS RXMS(std::string ang_rad);
MatS RYMS(std::string ang_rad);
MatS RZMS(std::string ang_rad);
MatS TXMS(std::string dist_in_mm);
MatS TYMS(std::string dist_in_mm);
MatS TZMS(std::string dist_in_mm);

std::string sinS(std::string ang);
std::string cosS(std::string ang);
std::string tanS(std::string ang);
std::string asinS(std::string ang);
std::string acosS(std::string ang);
std::string atan2S(std::string a, std::string b);

MatS generateForwardEquations(bool isprinted,int dof);
std::string generateForwardEquationsFunction(int dof);

bool isValidEquationsPrctice(const std::string& eq);

#endif