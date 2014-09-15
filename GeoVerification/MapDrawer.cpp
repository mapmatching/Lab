/*
* Last Updated at [2014/9/15 13:08] by wuhao
*/
#include "MapDrawer.h"


MapDrawer::MapDrawer()
{
	//GDI+ initializaton
	Gdiplus::GdiplusStartupInput gdiplusStartupInput;
	Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

	area = NULL;
	r_width = 0;
	r_height = 0;
}

MapDrawer::~MapDrawer()
{
	//GDI+ 收尾工作
	delete bm;
	delete bmData;
	Gdiplus::GdiplusShutdown(gdiplusToken);
}

//////////////////////////////////////////////////////////////////////////
///public part
//////////////////////////////////////////////////////////////////////////
void MapDrawer::setArea(double _minLat, double _maxLat, double _minLon, double _maxLon)
{
	if (area == NULL)
		area = new Area(_minLat, _maxLat, _minLon, _maxLon);
	else
		area->setArea(_minLat, _maxLat, _minLon, _maxLon);
}

void MapDrawer::setArea(Area* area)
{
	//[ATTENTION][MEMORY_LEAK]这里并没有将原area的内存给回收，可能会造成内存泄露
	this->area = area;
}

void MapDrawer::setResolution(int width)
{
	if (area->minLat == 0 && area->maxLat == 0 && area->maxLat == 0 && area->maxLon == 0)
	{
		printf("You should set the area first!\n");
		system("pause");
		exit(0);
	}
	r_width = width;
	double height = (area->maxLat - area->minLat) / (area->maxLon - area->minLon) * width;
	r_height = (int)height;
}

void MapDrawer::setResolution(int width, int height)
{
	r_width = width;
	r_height = height;
}

void MapDrawer::newBitmap()
{
	if (r_width == 0 || r_height == 0)
	{
		printf("You should set the area & resolution first!\n");
		system("pause");
		exit(0);
	}
	bm = new Gdiplus::Bitmap(r_width, r_height, PixelFormat32bppARGB);
}

void MapDrawer::lockBits()
{
	bmData = new Gdiplus::BitmapData;
	bm->LockBits(new Gdiplus::Rect(0, 0, r_width, r_height),
		Gdiplus::ImageLockModeRead | Gdiplus::ImageLockModeWrite, PixelFormat32bppARGB, bmData);
}

void MapDrawer::unlockBits()
{
	bm->UnlockBits(bmData);
}

void MapDrawer::drawPoint(Gdiplus::Color color, int x, int y)
{
	if (!inArea(x, y))
	{
		return;
	}
	byte* row = (byte*)bmData->Scan0 + (y * bmData->Stride);
	int x0 = x * 4;
	int x1 = x0 + 1;
	int x2 = x1 + 1;
	int x3 = x2 + 1;
	row[x0] = color.GetB();
	row[x1] = color.GetG();
	row[x2] = color.GetR();
	row[x3] = color.GetA();
}

void MapDrawer::drawPoint(Gdiplus::Color color, double lat, double lon)
{
	if (inArea(lat, lon))
	{
		Gdiplus::Point pt = geoToScreen(lat, lon);
		drawPoint(color, pt.X, pt.Y);
	}
}

void MapDrawer::drawBigPoint(Gdiplus::Color color, double lat, double lon)
{
	if (inArea(lat, lon))
	{
		Gdiplus::Point pt = geoToScreen(lat, lon);
		if (pt.X >= 1)
			drawPoint(color, pt.X - 1, pt.Y);
		if (pt.X <= r_width - 2)
			drawPoint(color, pt.X + 1, pt.Y);
		if (pt.Y >= 1)
			drawPoint(color, pt.X, pt.Y - 1);
		if (pt.Y <= r_height - 2)
			drawPoint(color, pt.X, pt.Y + 1);
		drawPoint(color, pt.X, pt.Y);
	}
}

void MapDrawer::drawBigPoint(Gdiplus::Color color, int x, int y)
{
	Gdiplus::Point pt = Gdiplus::Point(x, y);
	if (pt.X >= 1)
		drawPoint(color, pt.X - 1, pt.Y);
	if (pt.X <= r_width - 2)
		drawPoint(color, pt.X + 1, pt.Y);
	if (pt.Y >= 1)
		drawPoint(color, pt.X, pt.Y - 1);
	if (pt.Y <= r_height - 2)
		drawPoint(color, pt.X, pt.Y + 1);
	drawPoint(color, pt.X, pt.Y);
}

void MapDrawer::drawLine(Gdiplus::Color color, int x1, int y1, int x2, int y2)
{
	/*
	if (x1 == r_width)
	x1 = r_width - 1;
	if (x1 == -1)
	x1 = 0;
	if (x2 == r_width)
	x2 = r_width - 1;
	if (x2 == -1)
	x2 = 0;
	if (y1 == r_height)
	y1 = r_height - 1;
	if (y1 == -1)
	y1 = 0;
	if (y2 == r_height)
	y2 = r_height - 1;
	if (y2 == -1)
	y2 = 0;*/
	if (!inArea(x1, y1) && !inArea(x2, y2))
	{
		return;
	}
	if (abs(x1 - x2) >= abs(y1 - y2))
		bresenhamDrawLine_x(color, x1, y1, x2, y2);
	else
		bresenhamDrawLine_y(color, x1, y1, x2, y2);
}

void MapDrawer::drawLine(Gdiplus::Color color, double lat1, double lon1, double lat2, double lon2)
{
	if (!inArea(lat1, lon1) && !inArea(lat2, lon2))
	{
		return;
	}
	Gdiplus::Point pt1, pt2;
	pt1 = geoToScreen(lat1, lon1);
	pt2 = geoToScreen(lat2, lon2);
	drawLine(color, pt1.X, pt1.Y, pt2.X, pt2.Y);
}

void MapDrawer::drawBoldLine(Gdiplus::Color color, int x1, int y1, int x2, int y2)
{
	if (!inArea(x1, y1) && !inArea(x2, y2))
	{
		return;
	}
	drawLine(color, x1, y1, x2, y2);
	drawLine(color, x1 + 1, y1, x2 + 1, y2);
	drawLine(color, x1 - 1, y1, x2 - 1, y2);
	drawLine(color, x1, y1 + 1, x2, y2 + 1);
	drawLine(color, x1, y1 - 1, x2, y2 - 1);
}

void MapDrawer::drawBoldLine(Gdiplus::Color color, double lat1, double lon1, double lat2, double lon2)
{
	if (!inArea(lat1, lon1) && !inArea(lat2, lon2))
	{
		return;
	}
	Gdiplus::Point pt1, pt2, pt3, pt4, pt5, pt6;
	pt1 = geoToScreen(lat1, lon1);
	pt2 = geoToScreen(lat2, lon2);
	drawLine(color, pt1.X, pt1.Y, pt2.X, pt2.Y);
	pt3.X = pt1.X + 1;
	pt5.X = pt1.X - 1;
	pt5.Y = pt3.Y = pt1.Y;
	pt4.X = pt2.X + 1;
	pt6.X = pt2.X - 1;
	pt6.Y = pt4.Y = pt2.Y;
	drawLine(color, pt3.X, pt3.Y, pt4.X, pt4.Y);
	drawLine(color, pt5.X, pt5.Y, pt6.X, pt6.Y);
	pt3.Y = pt1.Y + 1;
	pt5.Y = pt1.Y - 1;
	pt5.X = pt3.X = pt1.X;
	pt4.Y = pt2.Y + 1;
	pt6.Y = pt2.Y - 1;
	pt6.X = pt4.X = pt2.X;
	drawLine(color, pt3.X, pt3.Y, pt4.X, pt4.Y);
	drawLine(color, pt5.X, pt5.Y, pt6.X, pt6.Y);
}

//Gdiplus::Color randomColor();

void MapDrawer::drawMap(Gdiplus::Color color, std::string mapFilePath)
{
	/************************************************************************/
	/* OpenStreetMap格式说明
	/* nodesOSM.txt: nodeId \t lat \t lon \n
	/* edgeOSM.txt: edgeId \t startNodeId \t endNodeId \t figureNodeCount \t figure1Lat \t figure1Lon \t ... figureNLat \t figureNLon \n
	/************************************************************************/

	std::ifstream ifs(mapFilePath);
	if (!ifs)
	{
		std::cout << "open " + mapFilePath + " error!\n";
		system("pause");
		return;
	}
	while (ifs)
	{
		int dummy, figureCount;
		ifs >> dummy >> dummy >> dummy >> figureCount;
		if (ifs.fail())
			break;
		double preLat, preLon, currentLat, currentLon;
		for (int i = 0; i < figureCount; i++)
		{
			if (i == 0)
			{
				ifs >> preLat >> preLon;
				continue;
			}
			ifs >> currentLat >> currentLon;
			drawLine(color, preLat, preLon, currentLat, currentLon);
			drawBigPoint(Gdiplus::Color::Black, preLat, preLon);
			preLat = currentLat;
			preLon = currentLon;
		}
		drawBigPoint(Gdiplus::Color::Black, preLat, preLon);
	}
	ifs.close();
}

void MapDrawer::saveBitmap(std::string fileName)
{

	CLSID pngClsid;
	GetEncoderClsid(L"image/png", &pngClsid);
	bm->Save(CharToWchar(fileName.c_str()), &pngClsid, NULL);
}

bool MapDrawer::inArea(double lat, double lon)
{
	return (lat > area->minLat && lat < area->maxLat && lon > area->minLon && lon < area->maxLon);
}

bool MapDrawer::inArea(int x, int y)
{
	return (x >= 0 && x < r_width && y >= 0 && y < r_height);
}

//北半球
void MapDrawer::zoomIn(int upperLeft_x, int upperLeft_y, int width, int height, int newR_width)
{
	double scaleY = (area->maxLat - area->minLat) / r_height;
	double scaleX = (area->maxLon - area->minLon) / r_width;
	double newMinLat, newMaxLat, newMinLon, newMaxLon;
	newMaxLat = area->maxLat - upperLeft_y * scaleY;
	newMinLat = newMaxLat - height * scaleY;
	newMinLon = area->minLon + upperLeft_x * scaleX;
	newMaxLon = newMinLon + width * scaleX;
	area->minLat = newMinLat;
	area->maxLat = newMaxLat;
	area->minLon = newMinLon;
	area->maxLon = newMaxLon;
	setResolution(newR_width);
}

Gdiplus::Point MapDrawer::geoToScreen(double lat, double lon)
{
	int x = (lon - area->minLon) / ((area->maxLon - area->minLon) / (double)r_width);
	int y = (area->maxLat - lat) / ((area->maxLat - area->minLat) / (double)r_height); //屏幕Y轴是向下递增的
	Gdiplus::Point pt(x, y);
	return pt;
}

std::pair<double, double> MapDrawer::screenToGeo(int screenX, int screenY)
{
	double scale = r_width / (area->maxLon - area->minLon);
	double lat = area->maxLat - (double)screenY / (double)r_height * (area->maxLat - area->minLat);
	double lon = (double)screenX / (double)r_width * (area->maxLon - area->minLon) + area->minLon;
	return std::make_pair(lat, lon);
}

//////////////////////////////////////////////////////////////////////////
///private part
//////////////////////////////////////////////////////////////////////////
int MapDrawer::GetEncoderClsid(const WCHAR* format, CLSID* pClsid)
{
	UINT  num = 0;          // number of image encoders
	UINT  size = 0;         // size of the image encoder array in bytes

	Gdiplus::ImageCodecInfo* pImageCodecInfo = NULL;

	Gdiplus::GetImageEncodersSize(&num, &size);
	if (size == 0)
		return -1;  // Failure

	pImageCodecInfo = (Gdiplus::ImageCodecInfo*)(malloc(size));
	if (pImageCodecInfo == NULL)
		return -1;  // Failure

	Gdiplus::GetImageEncoders(num, size, pImageCodecInfo);

	for (UINT j = 0; j < num; ++j)
	{
		if (wcscmp(pImageCodecInfo[j].MimeType, format) == 0)
		{
			*pClsid = pImageCodecInfo[j].Clsid;
			free(pImageCodecInfo);
			return j;  // Success
		}
	}

	free(pImageCodecInfo);
	return -1;  // Failure
}

void MapDrawer::bresenhamDrawLine_x(Gdiplus::Color color, int x1, int y1, int x2, int y2)
{
	int _x1, _y1, _x2, _y2;
	if (x1 < x2)
	{
		_x1 = x1;
		_y1 = y1;
		_x2 = x2;
		_y2 = y2;
	}
	else
	{
		_x1 = x2;
		_y1 = y2;
		_x2 = x1;
		_y2 = y1;
	}
	int x, y, dx, dy, e;
	dx = _x2 - _x1;
	dy = abs(_y2 - _y1);
	e = -dx;
	x = _x1;
	y = _y1;
	for (int i = 0; i <= dx; i++)
	{
		drawPoint(color, x, y);
		x++;
		e += 2 * dy;
		if (e >= 0)
		{
			if (_y1 > _y2)
				y--;
			else
				y++;
			e -= 2 * dx;
		}
	}
}

void MapDrawer::bresenhamDrawLine_y(Gdiplus::Color color, int x1, int y1, int x2, int y2)
{
	int _x1, _y1, _x2, _y2;
	if (y1 < y2)
	{
		_x1 = x1;
		_y1 = y1;
		_x2 = x2;
		_y2 = y2;
	}
	else
	{
		_x1 = x2;
		_y1 = y2;
		_x2 = x1;
		_y2 = y1;
	}
	int x, y, dx, dy, e;
	dy = _y2 - _y1;
	dx = abs(_x2 - _x1);
	e = -dy;
	y = _y1;
	x = _x1;
	for (int i = 0; i <= dy; i++)
	{
		drawPoint(color, x, y);
		y++;
		e += 2 * dx;
		if (e >= 0)
		{
			if (_x1 > _x2)
				x--;
			else
				x++;
			e -= 2 * dy;
		}
	}
}

wchar_t* MapDrawer::CharToWchar(const char* c)
{
	int len = MultiByteToWideChar(CP_ACP, 0, c, strlen(c), NULL, 0);
	wchar_t* m_wchar = new wchar_t[len + 1];
	MultiByteToWideChar(CP_ACP, 0, c, strlen(c), m_wchar, len);
	m_wchar[len] = '\0';
	return m_wchar;
}

/*输入：颜色，起始坐标，绘制的整型值*/
void MapDrawer::drawInt(Gdiplus::Color color, int x, int y, int value)
{
	//输入坐标为负时默认从0开始
	if (x < 0) x = 0;
	if (y < 0) y = 0;

	int iArray[100]; // 用来存储要绘制的整型值的每一位数字
	int i, j, k, Pos = 0, ald = 0;

	bool iNeg = false;
	//如果是负数，需要特别处理负号
	if (value < 0){
		iNeg = true;
		value *= -1;
		ald += 4;
	}
	//取出整型值的每一位数字，注意存放在数组中式倒序的
	while (value){
		iArray[Pos++] = value % 10;
		value /= 10;
	}
	//判断该串数字末尾位置是否超过画布最大长度。Y：重新调整起始坐标的位置，最终效果为最后一位数字紧贴画布尾部边缘
	if (x + Pos * 4 + ald  > r_width){
		x = r_width - Pos * 4 - ald;
	}
	if (y + 5 > r_height){
		y = r_height - 5;
	}
	//负数：输出负号
	if (iNeg){
		for (i = 0; i < 5; ++i){
			for (j = 0; j < 4; ++j){
				if (minus[i][j]){
					drawPoint(color, x + j, y + i);
				}
			}
		}
		x += 4;
	}
	//显示数字
	for (k = Pos - 1; k >= 0; --k){
		for (i = 0; i < 5; ++i){
			for (j = 0; j < 4; ++j){
				int numb = iArray[k];
				if (num[numb][i][j]){
					drawPoint(color, x + j, y + i);
				}
			}
		}
		x += 4;
	}
}
void MapDrawer::drawDouble(Gdiplus::Color color, int x, int y, double Value, int precision){
	int iArray[100];
	int i, j, k = precision, factor = 1, Pos = 0, ald = 0;
	bool iNeg = false, iPoint = false;

	while (k){
		factor *= 10;
		k--;
	}
	//负数需要特别处理
	if (Value < 0){
		Value *= -1;
		iNeg = true;
		ald += 4;
	}
	//将小数整数化
	double t = Value * factor;
	int value = int(t);
	//去掉小数点后面多余无用的0，如1.20000->1.2
	while (!(value % 10) && precision){
		value /= 10;
		precision--;
	}
	//将整数化后的数字取出，同样注意数组中为倒序
	while (value){
		iArray[Pos++] = value % 10;
		value /= 10;
	}
	//补充出小数点前的0，如0.07 的 0.0(7)
	while (Pos <= precision){
		iArray[Pos++] = 0;
	}
	//判断该串数字末尾位置是否超过画布最大长度。Y：重新调整起始坐标的位置，最终效果为最后一位数字紧贴画布尾部边缘
	if (x + 4 * Pos + 2 + ald > r_width){
		x = r_width - Pos * 4 - 2 - ald;
	}
	if (y + 5 > r_height){
		y = r_height - 5;
	}
	// 负数：输出负号
	if (iNeg){
		for (i = 0; i < 5; ++i){
			for (j = 0; j < 4; ++j){
				if (minus[i][j]){
					drawPoint(color, x + j, y + i);
				}
			}
		}
		x += 4;
	}
	//输出数字，并且在合适位置输出小数点
	for (k = Pos - 1; k >= 0; --k){
		for (i = 0; i<5; ++i){
			for (j = 0; j<4; ++j){
				int numb = iArray[k];
				if (num[numb][i][j]){
					if (iPoint) {
						drawPoint(color, x + j, y + i);
					}
					else{
						drawPoint(color, x + j, y + i);
					}
				}
			}
		}
		x += 4;
		if (k == precision){
			iPoint = true;
			for (i = 0; i<5; ++i){
				for (j = 0; j<2; ++j){
					if (point[i][j]){
						drawPoint(color, x + j, y + i);
					}
				}
			}
			x += 2;
		}
	}
}