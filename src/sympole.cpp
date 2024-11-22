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
        forward = forward * RZMS("theta0") * TZMS(linksS[0]);
    }
    if (dof > 1)
    {
        forward = forward * RYMS("theta1") * TZMS(linksS[1]);
    }
    if (dof > 2)
    {
        forward = forward * RYMS("theta2") * TZMS(linksS[2]);
    }
    if (dof > 3)
    {
        forward = forward * RZMS("theta3") * TZMS(linksS[3]);
    }
    if (dof > 4)
    {
        forward = forward * RYMS("theta4") * TZMS(linksS[4]);
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

/*


*/