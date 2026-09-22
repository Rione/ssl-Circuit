#include "MyMath.hpp"

const float _sin[91] = {
    SIN0,
    SIN1,
    SIN2,
    SIN3,
    SIN4,
    SIN5,
    SIN6,
    SIN7,
    SIN8,
    SIN9,
    SIN10,
    SIN11,
    SIN12,
    SIN13,
    SIN14,
    SIN15,
    SIN16,
    SIN17,
    SIN18,
    SIN19,
    SIN20,
    SIN21,
    SIN22,
    SIN23,
    SIN24,
    SIN25,
    SIN26,
    SIN27,
    SIN28,
    SIN29,
    SIN30,
    SIN31,
    SIN32,
    SIN33,
    SIN34,
    SIN35,
    SIN36,
    SIN37,
    SIN38,
    SIN39,
    SIN40,
    SIN41,
    SIN42,
    SIN43,
    SIN44,
    SIN45,
    SIN46,
    SIN47,
    SIN48,
    SIN49,
    SIN50,
    SIN51,
    SIN52,
    SIN53,
    SIN54,
    SIN55,
    SIN56,
    SIN57,
    SIN58,
    SIN59,
    SIN60,
    SIN61,
    SIN62,
    SIN63,
    SIN64,
    SIN65,
    SIN66,
    SIN67,
    SIN68,
    SIN69,
    SIN70,
    SIN71,
    SIN72,
    SIN73,
    SIN74,
    SIN75,
    SIN76,
    SIN77,
    SIN78,
    SIN79,
    SIN80,
    SIN81,
    SIN82,
    SIN83,
    SIN84,
    SIN85,
    SIN86,
    SIN87,
    SIN88,
    SIN89,
    SIN90};

float MyMath::sinDeg(int deg) {
    // 0~360の値に変換
    deg = normalizeDegrees(deg);
    int theta_cal = deg % 90;
    if (deg >= 90 && deg < 180) {
        theta_cal = 90 - theta_cal;
    }
    if (deg >= 270 && deg < 360) {
        theta_cal = 90 - theta_cal;
    }

    if (deg >= 0 && deg <= 90) {
        // 0~90 第一象限
        return _sin[theta_cal];
    } else if (deg > 90 && deg <= 180) {
        // 91~180 第二象限
        return _sin[theta_cal];
    } else if (deg > 180 && deg <= 270) {
        // 181~270 第三象限
        return -_sin[theta_cal];
    } else if (deg > 270 && deg < 360) {
        // 271~360 第四象限
        return -_sin[theta_cal];
    } else {
        return 0;
    }
}

float MyMath::cosDeg(int deg) {
    return sin(deg + 90);
}

float MyMath::tanDeg(int deg) {
    float rad = Radians(deg);
    if (cosDeg(rad) == 0)
        return 0;
    else
        return sinDeg(rad) / cosDeg(rad);
}

float MyMath::sin(float rad) {
    return sinDeg(Degrees(rad));
}

float MyMath::cos(float rad) {
    return cosDeg(Degrees(rad));
}

float MyMath::tan(float rad) {
    return tanDeg(Degrees(rad));
}

/// 2018/03 imo lab.
/// https://garchiving.com

int MyMath::atan2(int _y, int _x) {
    int x = abs(_x);
    int y = abs(_y);
    float z;
    bool c;

    c = y < x;
    if (c)
        z = (float)y / x;
    else
        z = (float)x / y;

    int a;
    // a = z * (-1556 * z + 6072);                     //2次曲線近似
    // a = z * (z * (-448 * z - 954) + 5894);          //3次曲線近似
    a = z * (z * (z * (829 * z - 2011) - 58) + 5741); // 4次曲線近似

    if (c) {
        if (_x > 0) {
            if (_y < 0) a *= -1;
        }
        if (_x < 0) {
            if (_y > 0) a = 18000 - a;
            if (_y < 0) a = a - 18000;
        }
    }

    if (!c) {
        if (_x > 0) {
            if (_y > 0) a = 9000 - a;
            if (_y < 0) a = a - 9000;
        }
        if (_x < 0) {
            if (_y > 0) a = a + 9000;
            if (_y < 0) a = -a - 9000;
        }
    }

    return a;
}

int MyMath::normalizeDegrees(int theta) {
    theta %= 360;
    if (theta < 0)
        theta += 360;
    return theta;
}

float MyMath::normalizeRadians(float theta) {
    while (theta < 0) {
        theta += TWO_PI;
    }
    while (theta >= TWO_PI) {
        theta -= TWO_PI;
    }
    return theta;
}

int MyMath::gapDegrees180(int deg1, int deg2) {
    int a = deg1 - deg2;
    while (a < 0)
        a += 360;
    while (a > 180)
        a -= 360;
    return a;
}

int MyMath::gapDegrees(int deg1, int deg2) {
    int a = normalizeDegrees(deg1) - normalizeDegrees(deg2);
    while (a < 0)
        a += 360;
    return a;
}

float MyMath::gapRadians180(float rad1, float rad2) {
    float a = rad1 - rad2;
    while (a < 0)
        a += TWO_PI;
    while (a > PI)
        a -= TWO_PI;
    return a;
}

float MyMath::gapRadians(float rad1, float rad2) {
    float a = normalizeRadians(rad1) - normalizeRadians(rad2);
    if (a < 0)
        a += TWO_PI;
    return a;
}

float MyMath::sqrt(float x) {
    float s, last;

    if (x < 0.0) {
        return 0;
    }

    if (x > 1.0) {
        s = x;
    } else {
        s = 1.0;
    }

    do {
        last = s;
        s = (x / s + s) / 2.0;
    } while (s < last);

    return last;
}