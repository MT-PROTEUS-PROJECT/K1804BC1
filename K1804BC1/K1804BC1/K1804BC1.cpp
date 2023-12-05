#include "K1804BC1.hpp"

#include <bitset>
#include <cmath>

K1804BC1::K1804BC1() : PQ(0)
{
    static_assert(sizeof(value_type) * 8 >= WORD_SIZE);
    MOD = static_cast<uint16_t>(std::pow(2, WORD_SIZE));
    std::fill(RON.begin(), RON.end(), 0);
}

VOID K1804BC1::setup(IINSTANCE* instance, IDSIMCKT* dsimckt)
{
    _instance = instance;

    vsm::model::init_pins(_instance, _pins_I, "I");
    vsm::model::init_pins(_instance, _pins_A, "A");
    vsm::model::init_pins(_instance, _pins_B, "B");
    vsm::model::init_pins(_instance, _pins_D, "D");
    vsm::model::init_pins(_instance, _pins_Y, "Y");
    vsm::model::init_pins(_instance, _pins_M, "M");

    _pins_F[0].init(_instance, "Z");
    _pins_F[1].init(_instance, "F3");
    _pins_F[2].init(_instance, "C4");
    _pins_F[3].init(_instance, "OVR");

    OE.init(_instance, "OE");
    CLK.init(_instance, "CLK");
    C0.init(_instance, "C0");

    PR0.init(_instance, "PR0");
    PR3.init(_instance, "PR7");
    PQ0.init(_instance, "PQ0");
    PQ3.init(_instance, "PQ7");
}

std::pair<K1804BC1::value_type, K1804BC1::value_type> K1804BC1::get_operands()
{
    auto operands = vsm::model::make_number(std::views::counted(_pins_I.begin(), 3));
    auto ronA = vsm::model::make_number(_pins_A);
    auto ronB = vsm::model::make_number(_pins_B);
    auto data = vsm::model::make_number(_pins_D);

    value_type r = 0;
    value_type s = 0;

    switch (operands)
    {
    case 0:
        r = RON[ronA];
        s = PQ;
        break;
    case 1:
        r = RON[ronA];
        s = RON[ronB];
        break;
    case 2:
        r = 0;
        s = PQ;
        break;
    case 3:
        r = 0;
        s = RON[ronB];
        break;
    case 4:
        r = 0;
        s = RON[ronA];
        break;
    case 5:
        r = data;
        s = RON[ronA];
        break;
    case 6:
        r = data;
        s = PQ;
        break;
    case 7:
        r = data;
        s = 0;
        break;
    default:
        break;
    }

    return { r, s };
}

K1804BC1::value_type K1804BC1::get_result(value_type r, value_type s, bool& ovr, bool& c4, bool& f3, bool& z)
{
    auto func = vsm::model::make_number(std::views::counted(_pins_I.begin() + 3, 3));
    auto c0 = C0->isactive();

    value_type res = 0;
    bool sub = false;
    switch (func)
    {
    case 0:
        res = r + s + c0;
        break;
    case 1:
        res = s - r - 1 + c0;
        sub = true;
        r = (~r & ((1 << WORD_SIZE) - 1));
        break;
    case 2:
        res = r - s - 1 + c0;
        sub = true;
        s = (~s & ((1 << WORD_SIZE) - 1));
        break;
    case 3:
        res = r | s;
        break;
    case 4:
        res = r & s;
        break;
    case 5:
        res = ~r & s;
        break;
    case 6:
        res = r ^ s;
        break;
    case 7:
        res = ~(r ^ s);
        break;
    default:
        break;
    }

    bool _c0 = (func == 1 || func == 2);

    value_type p = r | s;
    value_type g = r & s;

    std::bitset<WORD_SIZE> bp(p);
    std::bitset<WORD_SIZE> bg(g);

    std::array<bool, WORD_SIZE> c;
    c[0] = bg[0] || (bp[0] && _c0);
    for (size_t i = 1; i < WORD_SIZE; ++i)
    {
        c[i] = bg[i] || (bp[i] && c[i - 1]);
    }
    c4 = c.back();
    res %= MOD;

    f3 = (res >= MOD / 2);
    ovr = (r < MOD / 2 && s < MOD / 2 && res >= MOD / 2 && !sub) || (r >= MOD / 2 && s >= MOD / 2 && res < MOD / 2 && !sub);
    z = res == 0;
    return res;
}

void K1804BC1::write_result(ABSTIME time, value_type res, bool ovr, bool c4, bool f3, bool z)
{
    auto ronA = vsm::model::make_number(_pins_A);
    auto ronB = vsm::model::make_number(_pins_B);
    auto out = vsm::model::make_number(std::views::counted(_pins_I.begin() + 6, 3));
    auto y = res;

    switch (out)
    {
    case 0:
        PQ = res;
        break;
    case 1:
        break;
    case 2:
        y = RON[ronA];
        RON[ronB] = res;
        break;
    case 3:
        RON[ronB] = res;
        break;
    case 4:
    case 5:
    case 6:
    case 7:
        set_shift(time, out, res, ronB);
        break;
    default:
        break;
    }

    for (size_t i = 0; i < _pins_Y.size(); _pins_Y[i].set(time, 500, y % 2 ? SHI : SLO), y /= 2, ++i);

    _pins_F[0].set(time, 500, z ? SHI : SLO);
    _pins_F[1].set(time, 500, f3 ? SHI : SLO);
    _pins_F[2].set(time, 500, c4 ? SHI : SLO);
    _pins_F[3].set(time, 500, ovr ? SHI : SLO);
}

void K1804BC1::set_shift(ABSTIME time, uint32_t out, uint32_t res, uint32_t ronB)
{
    auto m = vsm::model::make_number(_pins_M);

    auto pr0_prev = (RON[ronB] & (1u));
    auto pr7_prev = (RON[ronB] & (1u << (WORD_SIZE - 1)));

    auto pq0_prev = (PQ & (1u));
    auto pq7_prev = (PQ & (1u << (WORD_SIZE - 1)));

    bool right_shift = true;
    switch (out)
    {
    case 4:
        PQ >>= 1;
    case 5:
        RON[ronB] = (res >> 1);
        break;
    case 6:
        PQ <<= 1;
    case 7:
        RON[ronB] = (res << 1);
        right_shift = false;
        break;
    }

    auto pr0_cur = (RON[ronB] & (1u));
    auto pr7_cur = (RON[ronB] & (1u << (WORD_SIZE - 1)));

    auto pq0_cur = (PQ & (1u));
    auto pq7_cur = (PQ & (1u << (WORD_SIZE - 1)));

    if (m == 1)
    {
        if (right_shift)
        {
            pr7_cur = pr0_prev;
            pq7_cur = pq0_prev;
        }
        else
        {
            pr0_cur = pr7_prev;
            pq0_cur = pq7_prev;
        }
    }
    else if (m == 2)
    {
        if (right_shift)
        {
            pq7_cur = pr0_prev;
            pr7_cur = pq0_prev;
        }
        else
        {
            pq0_cur = pr7_prev;
            pr0_cur = pq7_prev;
        }
    }
    else if (m == 3)
    {
        if (right_shift)
        {
            pq7_cur = pr0_prev;
        }
        else
        {
            pr0_cur = pq7_prev;
        }
    }

    RON[ronB] &= (1u << WORD_SIZE) - 2;
    RON[ronB] |= pr0_cur;

    RON[ronB] &= (1u << WORD_SIZE - 1) - 1;
    if (pr7_cur)
        RON[ronB] |= (1u << WORD_SIZE - 1);

    PQ &= (1u << WORD_SIZE) - 2;
    PQ |= pq0_cur;

    PQ &= (1u << WORD_SIZE - 1) - 1;
    if (pq7_cur)
        PQ |= (1u << WORD_SIZE - 1);

    PR0.set(time, 500, pr0_cur ? SHI : SLO);
    PR3.set(time, 500, pr7_cur ? SHI : SLO);
    PQ0.set(time, 500, pq0_cur ? SHI : SLO);
    PQ3.set(time, 500, pq7_cur ? SHI : SLO);
}

VOID K1804BC1::simulate(ABSTIME time, DSIMMODES mode)
{
    if (OE->isactive())
    {
        std::fill(RON.begin(), RON.end(), 0);
        PQ = 0;
        return;
    }

    if (!CLK->isposedge())
        return;

    bool ovr = 0, f3 = 0, z = 0, c4 = 0;
    auto [r, s] = get_operands();

    auto res = get_result(r, s, ovr, c4, f3, z);

    write_result(time, res, ovr, c4, f3, z);
}

extern "C"
{
    IDSIMMODEL __declspec(dllexport)* createdsimmodel(CHAR* device, ILICENCESERVER* license_server)
    {
        return license_server->authorize(K1804BC1::MODEL_KEY) ? new K1804BC1 : nullptr;
    }

    VOID __declspec(dllexport) deletedsimmodel(IDSIMMODEL* model)
    {
        delete static_cast<K1804BC1*>(model);
    }
}
