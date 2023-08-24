#include "K1804BC1.hpp"

#include <bitset>

K1804BC1::K1804BC1() : PQ(0)
{
	static_assert(sizeof(value_type) * 8 >= WORD_SIZE);
	std::fill(RON.begin(), RON.end(), 0);
}

VOID K1804BC1::setup(IINSTANCE *instance, IDSIMCKT *dsimckt)
{
	_instance = instance;

	vsm::model::init_pins(_instance, _pins_I, "I");
	vsm::model::init_pins(_instance, _pins_A, "A");
	vsm::model::init_pins(_instance, _pins_B, "B");
	vsm::model::init_pins(_instance, _pins_D, "D");
	vsm::model::init_pins(_instance, _pins_Y, "Y");
	
	_pins_F[0].init(_instance, "Z");
	_pins_F[1].init(_instance, "F3");
	_pins_F[2].init(_instance, "C4");
	_pins_F[3].init(_instance, "OVR");

	OE.init(_instance, "OE");
	CLK.init(_instance, "CLK");
	C0.init(_instance, "C0");
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

	return {r, s};
}

int modulo(int a, int m)
{
	a %= m;
	if (a < 0)
	{
		a += m;
	}
	return a;
}

int K1804BC1::get_result(value_type r, value_type s, bool &ovr, bool &c4, bool &f3, bool &z)
{
	auto func = vsm::model::make_number(std::views::counted(_pins_I.begin() + 3, 3));
	auto c0 = C0->isactive();

	int res = 0;
	bool sub = false;
	switch (func)
	{
	case 0:
		res = r + s + c0;
		break;
	case 1:
		res = s - r - 1 + c0;
		sub = true;
		r = (~r & ((1 << 4) - 1));
		break;
	case 2:
		res = r - s - 1 + c0;
		sub = true;
		s = (~s & ((1 << 4) - 1));
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

	uint8_t p = r | s;
	uint8_t g = r & s;

	std::bitset<4> bp(p);
	std::bitset<4> bg(g);

	bool c1 = bg[0] || (bp[0] && _c0);
	bool c2 = bg[1] || (bp[1] && c1);
	bool c3 = bg[2] || (bp[2] && c2);
	c4 = bg[3] || (bp[3] && c3);
	res = modulo(res, 16);

	f3 = res >= 8;
	ovr = (r < 8 && s < 8 && res >= 8 && !sub) || (r >= 8 && s >= 8 && res < 8 && !sub);
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
		PQ /= 2;
		RON[ronB] = res / 2;
		break;
	case 5:
		RON[ronB] = res / 2;
		break;
	case 6:
		PQ *= 2;
		RON[ronB] = 2 * res;
		break;
	case 7:
		RON[ronB] = 2 * res;
		break;
	default:
		break;
	}

	for (size_t i = 0; i < _pins_Y.size(); _pins_Y[i].set(time, 500, y % 2 ? SHI : SLO), y /= 2, ++i);

	_pins_F[0].set(time, 500, z ? SHI : SLO);
	_pins_F[1].set(time, 500, f3 ? SHI : SLO);
	_pins_F[2].set(time, 500, ovr ? SHI : SLO);
	_pins_F[3].set(time, 500, c4 ? SHI : SLO);
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
	auto [r, s]	= get_operands();

	auto res = get_result(r, s, ovr, c4, f3, z);

	write_result(time, res, ovr, c4, f3, z);
}

extern "C"
{
	IDSIMMODEL __declspec(dllexport) *createdsimmodel(CHAR *device, ILICENCESERVER *license_server)
	{
		return license_server->authorize(K1804BC1::MODEL_KEY) ? new K1804BC1 : nullptr;
	}

	VOID __declspec(dllexport) deletedsimmodel(IDSIMMODEL *model)
	{
		delete static_cast<K1804BC1 *>(model);
	}
}
