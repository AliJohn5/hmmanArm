#include "hummanArm/sympole.hpp"

void strip(std::string &_s)
{
    int cnt = 0;
    for (auto &c : _s)
    {
        if (c == ' ' || c == '\n')
        {
            ++cnt;
        }
    }

    std::string ans(_s.size() - cnt, '*');

    cnt = 0;

    for (auto &c : _s)
    {
        if (c != ' ' && c != '\n')
        {
            ans[cnt] = c;
            ++cnt;
        }
    }

    _s = ans;
}

std::string sinS(std::string ang)
{
    strip(ang);
    return "sin(" + ang + ")";
}

std::string cosS(std::string ang)
{
    strip(ang);
    return "cos(" + ang + ")";
}

std::string tanS(std::string ang)
{
    strip(ang);
    return "tan(" + ang + ")";
}

std::string asinS(std::string ang)
{
    strip(ang);
    return "asin(" + ang + ")";
}

std::string acosS(std::string ang)
{
    strip(ang);
    return "acos(" + ang + ")";
}

std::string atan2S(std::string a, std::string b)
{
    strip(a);
    strip(b);

    return "atan2(" + a + "," + b + ")";
}

std::string sqrtS(std::string ang)
{
    strip(ang);
    return "sqrt(" + ang + ")";
}

MatS RXMS(std::string ang_rad)
{
    return {
        VecS({"1", "0", "0", "0"}),
        VecS({"0", cosS(ang_rad), "-" + sinS(ang_rad), "0"}),
        VecS({"0", sinS(ang_rad), cosS(ang_rad), "0"}),
        VecS({"0", "0", "0", "1"})};
}

MatS RYMS(std::string ang_rad)
{
    return {
        VecS({cosS(ang_rad), "0", sinS(ang_rad), "0"}),
        VecS({"0", "1", "0", "0"}),
        VecS({"-" + sinS(ang_rad), "0", cosS(ang_rad), "0"}),
        VecS({"0", "0", "0", "1"})};
}

MatS RZMS(std::string ang_rad)
{
    return {
        VecS({cosS(ang_rad), "-" + sinS(ang_rad), "0", "0"}),
        VecS({sinS(ang_rad), cosS(ang_rad), "0", "0"}),
        VecS({"0", "0", "1", "0"}),
        VecS({"0", "0", "0", "1"})};
}

MatS TXMS(std::string dist_in_mm)
{
    return {VecS({"1", "0", "0", dist_in_mm}),
            VecS({"0", "1", "0", "0"}),
            VecS({"0", "0", "1", "0"}),
            VecS({"0", "0", "0", "1"})};
}

MatS TYMS(std::string dist_in_mm)
{
    return {
        VecS({"1", "0", "0", "0"}),
        VecS({"0", "1", "0", dist_in_mm}),
        VecS({"0", "0", "1", "0"}),
        VecS({"0", "0", "0", "1"})};
}

MatS TZMS(std::string dist_in_mm)
{
    return {VecS({"1", "0", "0", "0"}),
            VecS({"0", "1", "0", "0"}),
            VecS({"0", "0", "1", dist_in_mm}),
            VecS({"0", "0", "0", "1"})};
}

std::string prodTwoString(std::string &a, std::string &b)
{
    strip(a);
    strip(b);

    if (a == "0" || a == "")
        return "";

    if (a == "(0)" || a == "()")
        return "";

    if (b == "0" || b == "")
        return "";

    if (b == "(0)" || b == "()")
        return "";

    if (a == "1" || a == "(1)")
        return b;

    if (b == "1" || b == "(1)")
        return a;

    return "(" + a + "*" + b + ")";
}

std::string sumTwoString(std::string &a, std::string &b)
{
    strip(a);
    strip(b);
    if (a == "0" || a == "" || a == "(0)" || a == "()")
    {
        if (b == "0" || b == "" || b == "(0)" || b == "()")
        {
            return "";
        }
        return b;
    }

    if (b == "0" || b == "" || b == "(0)" || b == "()")
        return a;

    return "(" + a + "+" + b + ")";
}

MatS operator*(const MatS &a, const MatS &b)
{
    MatS res = {
        VecS({"", "", "", ""}),
        VecS({"", "", "", ""}),
        VecS({"", "", "", ""}),
        VecS({"", "", "", ""}),
    };

    MatS a1 = a, b1 = b;

    for (int i = 0; i < a1.size(); ++i)
    {
        for (int j = 0; j < b1[0].size(); ++j)
        {
            for (int k = 0; k < b1.size(); ++k)
            {
                std::string prod = prodTwoString(a1[i][k], b1[k][j]);
                res[i][j] = sumTwoString(res[i][j], prod);
            }
        }
    }

    return res;
}
bool isValidEquationsPrctice(const std::string &eq)
{

    int cntstart = 0;

    for (const auto &c : eq)
    {
        if (c == '(')
            ++cntstart;
        if (c == ')')
        {
            if (cntstart < 1)
                return false;
            else
                --cntstart;
        }
    }
    return true;
}

MatS generateForwardEquations(bool isprinted, int dof)
{

    MatS forward = IDENS;

    if (dof > 0)
    {
        forward = forward * RZMS("theta0") * TZMS("links[0]");
    }
    if (dof > 1)
    {
        forward = forward * RYMS("theta1") * TZMS("links[1]");
    }
    if (dof > 2)
    {
        forward = forward * RYMS("theta2") * TZMS("links[2]");
    }
    if (dof > 3)
    {
        forward = forward * RZMS("theta3") * TZMS("links[3]");
    }
    if (dof > 4)
    {
        forward = forward * RYMS("theta4") * TZMS("links[4]");
    }

    if (isprinted)
    {
        std::cout << "Position is: \n";
        std::cout << "\tx =\t" << forward[0][3] << ";\n\n";
        std::cout << "\ty =\t" << forward[1][3] << ";\n\n";
        std::cout << "\tz =\t" << forward[2][3] << ";\n\n";

        std::cout << "Rotation is: \n";
        for (size_t i = 0; i < 3; ++i)
        {
            for (size_t j = 0; j < 3; ++j)
            {
                std::cout << "\tR" << std::to_string(i) << std::to_string(j) << " =\t" << forward[i][j] << ";\n\n";
            }
        }
    }

    return forward;
}

MatS generateForwardEquations(std::vector<std::string> ang)
{
    MatS forward = IDENS;
    int dof = ang.size();

    if (dof > 0)
    {
        forward = forward * RZMS(ang[0]) * TZMS("links[0]");
    }
    if (dof > 1)
    {
        forward = forward * RYMS(ang[1]) * TZMS("links[1]");
    }
    if (dof > 2)
    {
        forward = forward * RYMS(ang[2]) * TZMS("links[2]");
    }
    if (dof > 3)
    {
        forward = forward * RZMS(ang[3]) * TZMS("links[3]");
    }
    if (dof > 4)
    {
        forward = forward * RYMS(ang[4]) * TZMS("links[4]");
    }

    return forward;
}

std::string generateForwardEquationsFunction(int dof)
{
    std::string ans = "Mat forwardUsingEquations(Ang ang_rad){\n";
    MatS eq = generateForwardEquations(false, dof);

    ans += "\tMat ans;\n";

    ans += "\tdouble sinV[5];\n";
    ans += "\tdouble cosV[5];\n";

    ans += "\tfor(int i = 0 ; i < 5 ; ++i){\n";
    ans += "\t\tsinV[i] = sin(ang_rad[i]);\n";
    ans += "\t\tcosV[i] = cos(ang_rad[i]);\n";
    ans += "\t}\n";

    for (size_t i = 0; i < 4; i++)
    {
        for (size_t j = 0; j < 4; j++)
        {
            if (eq[i][j] == "")
                eq[i][j] = "0";
            ans += "\tans[" + std::to_string(i) + "][" + std::to_string(j) + "] = " + eq[i][j] + ";\n";
        }
    }

    ans += "\treturn ans;\n";

    ans += "}";

    std::string ans1;
    int i;

    for (i = 0; i < ans.size() - 9; ++i)
    {
        std::string temp;
        for (size_t j = 0; j < 9; j++)
        {
            temp += ans[i + j];
        }

        if (temp == "sin(theta")
        {
            ans1 += "sinV[";
            ans1.push_back(ans[i + 9]);
            ans1.push_back(']');
            i += 10;
        }

        else if (temp == "cos(theta")
        {
            ans1 += "cosV[";
            ans1.push_back(ans[i + 9]);
            ans1.push_back(']');
            i += 10;
        }
        else
        {
            ans1 += ans[i];
        }
    }

    for (; i < ans.size(); ++i)
    {
        ans1 += ans[i];
    }

    if (isValidEquationsPrctice(ans1))
        return ans1;
    return "ERROR";
}

std::string generateInverseEquationsFunction()
{
    MatS mat = {
        VecS({"mat[0][0]", "mat[0][1]", "mat[0][2]", "mat[0][3]"}),
        VecS({"mat[1][0]", "mat[1][1]", "mat[1][2]", "mat[1][3]"}),
        VecS({"mat[2][0]", "mat[2][1]", "mat[2][2]", "mat[2][3]"}),
        VecS({"mat[3][0]", "mat[3][1]", "mat[3][2]", "mat[3][3]"})};

    MatS mat4 = mat * TZMS("-" + linksS[4]);
    std::string s = "std::vector<Ang> inverseUsingEquations(Mat mat)\n{\n\tstd::vector<Ang> ans(2);\n";

    s += "\tans[0][0] = ans[1][0] = " + atan2S(mat4[1][3], mat4[0][3]) + ";\n\n";

    std::string h1 = prodTwoString(mat4[0][3], mat4[0][3]);
    std::string h2 = prodTwoString(mat4[1][3], mat4[1][3]);
    std::string h3 = sumTwoString(h1, h2);
    std::string _r = sqrtS(h3);
    std::string _s = "(" + mat4[2][3] + "-links[0])";

    std::string a11 = "(links[2]+links[3])";
    std::string a21 = "(links[1]*links[1])";
    std::string a31 = "(links[1])";
    std::string a41 = prodTwoString(_r, _r);
    std::string a51 = prodTwoString(_s, _s);
    std::string a61 = prodTwoString(a11, a11);
    std::string a71 = sumTwoString(a41, a51);
    std::string a81 = "(" + a71 + "-" + a61 + "-" + a21 + ")";

    std::string a91 = "(2*" + a11 + "*" + a31 + ")";
    std::string a110 = "(" + a81 + "/" + a91 + ")";

    s += "\tans[0][2] = " + acosS(a110) + ";\n\n";
    s += "\tans[1][2] = -" + acosS(a110) + ";\n\n";

    std::string b01 = sinS("M_PI-ans[0][2]");
    std::string b11 = sinS("M_PI-ans[1][2]");
    std::string b03 = atan2S(_s, _r);
    std::string b04 = sqrtS(a51 + "+" + a41);

    std::string b05 = "(" + asinS("(" + a11 + "*" + b01 + ")/" + b04) + "+" + b03 + ")";
    std::string b06 = "(" + asinS("(" + a11 + "*" + b11 + ")/" + b04) + "+" + b03 + ")";

    s += "\tans[0][1] = M_PI_2-" + b05 + ";\n\n";

    s += "\tans[1][1] = M_PI_2-" + b06 + ";\n\n";

    MatS _mat20 = generateForwardEquations({"ans[0][0]", "ans[0][1]", "ans[0][2]"});
    MatS _mat21 = generateForwardEquations({"ans[1][0]", "ans[1][1]", "ans[1][2]"});

    std::swap(_mat20[1][0], _mat20[0][1]);
    std::swap(_mat20[2][0], _mat20[0][2]);
    std::swap(_mat20[1][2], _mat20[2][1]);

    std::swap(_mat21[1][0], _mat21[0][1]);
    std::swap(_mat21[2][0], _mat21[0][2]);
    std::swap(_mat21[1][2], _mat21[2][1]);

    MatS _d1 = _mat20 * mat4;
    MatS _d2 = _mat21 * mat4;

    s += "\tans[0][3]=" + atan2S(_d1[1][2], _d1[0][2]) + ";\n\n";
    s += "\tans[1][3]=" + atan2S(_d2[1][2], _d2[0][2]) + ";\n\n";

    s += "\tans[0][4]=" + acosS(_d1[2][2]) + ";\n\n";
    s += "\tans[1][4]=" + acosS(_d2[2][2]) + ";\n\n";

    s += "\treturn ans;\n}";
    if (isValidEquationsPrctice(s))
        return s;
    else
        return "ERROR";
}
/*


*/