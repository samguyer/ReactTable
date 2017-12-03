#include <adel.h>
#include "SSD1306Wire.h"
class Pattern;
#include <FastLED.h>

// ===== Global configuration =====================================

#define ROTARY_A_PIN 19
#define ROTARY_B_PIN 18
#define BUTTON_PIN 32

#define IR_CHANNEL_BIT_0 2  // 12
#define IR_CHANNEL_BIT_1 0  // 14
#define IR_CHANNEL_BIT_2 4  // 27
#define IR_CHANNEL_BIT_3 16 // 26

int IR_INPUT[] = { 35, 34, 39, 36 };

#define SDA_PIN 21
#define SCL_PIN 22

#define X_PIN 32
#define Y_PIN 33
#define Z_PIN 25

#define MIC_PIN 13

#define LED_PIN 17

#define NUM_CELLS 61
#define LEDS_PER_CELL 12
#define NUM_LEDS (NUM_CELLS * LEDS_PER_CELL)
#define FRAMES_PER_SECOND 60

#define CELL_COLS 12
#define CELL_ROWS 12

#define IMAGE_WIDTH_SCALE  5
#define IMAGE_HEIGHT_SCALE 3
#define IMAGE_WIDTH  ((CELL_COLS+2) * IMAGE_WIDTH_SCALE)
#define IMAGE_HEIGHT ((CELL_ROWS+2) * IMAGE_HEIGHT_SCALE)

// ===== OLED Display =============================================

SSD1306Wire  display(0x3c, SDA_PIN, SCL_PIN);

// ===== LED strip information ====================================

CRGB g_LEDs[NUM_LEDS+12];

uint8_t g_Brightness = 50;

#define COLOR_ORDER GRB
#define CHIPSET     WS2812

CRGBPalette16 g_palette[] = 
    { RainbowColors_p, 
      CloudColors_p,
      LavaColors_p,
      OceanColors_p,
      ForestColors_p,
      PartyColors_p,
      HeatColors_p };

// ===== Cells ====================================================

class Cell;

/** Pattern
 *
 *  This class is the abstract superclass of each specific kind of display
 *  pattern. Each cell owns one instance of current pattern class. The two
 *  important methods are clone and render. Clone is used by the cell class
 *  to instantiate a pattern instance for each cell (since it may need its
 *  own state). The render method is the main "drawing" method. It is
 *  called by the owning cell for each frame of the animation.
 */
class Pattern
{
friend class Cell;
    
protected:
    Cell * m_cell;
    
public:
    Pattern()
        : m_cell(0)
    {}

    virtual ~Pattern() {}
    virtual Pattern * clone() =0;
    virtual void init() {}
    virtual void render(uint8_t level, uint32_t delta_t) =0;

private:
    void setCell(Cell * cell) {
        m_cell = cell;
    }
};

/** Cell
 *
 *  Each cell represents one IR sensor and an LED ring. The Cell class
 *  contains methods for reading the IR sensor value and managing tasks
 *  such as callibration and smoothing. The cell class also contains many
 *  different methods for writing color values to the LED ring. These
 *  methods are called by the current Pattern instance to render its
 *  particular animation.
 */
class Cell
{
private:
    // -- Global parameters to control IR decay
    static bool      m_decay;
    static uint8_t   m_up_decay;
    static uint8_t   m_down_decay;

private:
    // -- Cell position
    int       m_x;
    int       m_y;
    
    // -- IR configuration
    int       m_ir_input;
    int       m_ir_channel;
    int       m_ir_max;
    int       m_ir_min;
    uint16_t  m_ir_history[8];
    int       m_ir_cur;
    uint16_t  m_ir_running_sum;
    uint8_t   m_level;

    // -- LED ring information
    int             m_led_index;
    uint16_t        m_translate;
    CRGBPalette16   m_palette;

    // -- Visual pattern
    Pattern *       m_pattern;

    // -- Visual options
    uint8_t         m_fade_by;
    
public:
    Cell(int ir_input, int ir_channel, int X, int Y, int led_pos )
        : m_x(X),
          m_y(Y),
          m_ir_input(ir_input),
          m_ir_channel(ir_channel),
          m_ir_cur(0),
          m_led_index(led_pos * LEDS_PER_CELL), 
          m_translate(0),
          m_palette(RainbowColors_p),
          m_pattern(0),
          m_fade_by(0)
    {
        CRGB yellow = CRGB::Yellow;
        setAllLEDs(yellow);
        FastLED.show();
        calibrateIR();
        CRGB green = CRGB::Green;
        setAllLEDs(green);
        FastLED.show();
    }

    // ----- Getters and setters -----

    // -- Set the palette explicitly
    void setPalette(CRGBPalette16 new_palette) {
        m_palette = new_palette;
    }

    // -- Set the palette by index
    void setPaletteNum(int which) {
        if (which >= 0 and which < 6)  m_palette = g_palette[which];
        else m_palette = RainbowColors_p;
    }

    CRGBPalette16 getPalette() const { return m_palette; }

    void setFadeBy(uint8_t fade) { m_fade_by = fade; }
    uint8_t getFadeBy() const { return m_fade_by; }

    void setTranslation(uint16_t trans) { m_translate = trans; }
    uint16_t getTranslation(void) const { return m_translate; }

    uint16_t getX() const { return m_x; }
    uint16_t getY() const { return m_y; }

    // ----- Rendering methods -----

    /* Get and Set raw LEDs
     * Access the ring without any translation or interpolation.
     */
    CRGB& getRawLED(int local_index) {
        if (local_index < 0) local_index = 0;
        if (local_index >= LEDS_PER_CELL) local_index = LEDS_PER_CELL - 1;
        return g_LEDs[m_led_index + local_index];
    }

    void setRawLED(int local_index, CRGB color) {
        if (local_index < 0) local_index = 0;
        if (local_index >= LEDS_PER_CELL) local_index = LEDS_PER_CELL - 1;
        g_LEDs[m_led_index + local_index] = color;
    }

    /* Set a pixel
     *  
     * The position is expressed as a 16-bit fixed-precision number that maps
     * to whatever number of actual LEDs there are in the strip/ring. This
     * function dithers the adjacent pixels to create a smooth transition.
     * It needs to be fast!
     */
    void setPixel(uint16_t pos_in, uint8_t hue, uint8_t brightness = 255) {
        // -- Apply translation
        //    This might overflow, which supports LED rings
        uint16_t pos = pos_in + m_translate;
      
        // -- Get the color from the palette
        CRGB color = ColorFromPalette(m_palette, hue);
        if (brightness < 255) color.nscale8_video(brightness);

        // -- Scale the position from 0-0xFFFF to 0-m_num_leds
        //    but keep the fractional part. The result is a 16-bit
        //    value where the high 8 bits are in [0,num_leds] and
        //    the low 8 bits are the faction in [0,0xFF]
        uint32_t pos32 = (uint32_t) pos;
        uint16_t scaled_pos = (pos32 * LEDS_PER_CELL) >> 8;

        // -- Break the position into the whole and fractional parts. The
        //    fractional part is represented by the brightness of two 
        //    adjacent LEDs.
        int indexA = scaled_pos >> 8;
        uint8_t offsetA = scaled_pos & 0xFF;

        // -- Pixels can straddle two LEDs, so compute the other partial LED.
        int indexB; 
        if (indexA == 0) indexB = LEDS_PER_CELL - 1;
        else indexB = indexA - 1;
        uint8_t offsetB = 0xFF - offsetA;

        // -- Scale the brightness and assign the LEDs
        if (offsetA > 10) nblend(g_LEDs[m_led_index + indexA], color, offsetA);
        if (offsetB > 10) nblend(g_LEDs[m_led_index + indexB], color, offsetB);
    }

    /** Set an LED
     *  
     *  Set an LED value based on LED index rather than abstract position.
     *  Other than possible translation of coordinates, this should end
     *  up setting the color on a particular real LED device.
     */
    void setLED(uint8_t led, uint8_t hue, uint8_t brightness = 255) {
        uint16_t pos = ((((uint16_t) led) << 8) / LEDS_PER_CELL) << 8;
        setPixel(pos, hue, brightness);
    }

    /** Set all LEDs
     *  
     *  Set all the LEDs to a particular color. Since they are all the
     *  same, there is no need to use set_pixel.
     */
    void setAllLEDs(CRGB& color) {
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            g_LEDs[m_led_index + i] = color;
        }
    }

    /** Set all LEDs
     *  
     *  Set all the LEDs to a particular hue.
     */
    void setAllLEDs(uint8_t hue, uint8_t brightness = 255) {
        CRGB color = ColorFromPalette(m_palette, hue);
        if (brightness < 255) color.nscale8_video(brightness);
        setAllLEDs(color);
    }

    /** Fade all LEDs
     *  
     */
    void fadeToBlackBy(uint8_t howmuch) {
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            g_LEDs[m_led_index + i].fadeToBlackBy(howmuch);
        }
    }

    // ----- IR sensing -----

    /** Read raw IR value
     *
     *  Read from the channel assigned to this cell. 
     */
    uint16_t readIR()
    {
        uint8_t channel = m_ir_channel;

        // -- Select the channel
        if (channel & 0x1) digitalWrite(IR_CHANNEL_BIT_0, HIGH);
        else digitalWrite(IR_CHANNEL_BIT_0, LOW);

        if (channel & 0x2) digitalWrite(IR_CHANNEL_BIT_1, HIGH);
        else digitalWrite(IR_CHANNEL_BIT_1, LOW);

        if (channel & 0x4) digitalWrite(IR_CHANNEL_BIT_2, HIGH);
        else digitalWrite(IR_CHANNEL_BIT_2, LOW);

        if (channel & 0x8)digitalWrite(IR_CHANNEL_BIT_3, HIGH);
        else digitalWrite(IR_CHANNEL_BIT_3, LOW);

        // -- Finally, read the analog value
        uint16_t val = analogRead(IR_INPUT[m_ir_input]);
        return val;
    }

    /** Calibrate IR
     *
     *  Read a series of values, using the average value as the max
     */
    void calibrateIR()
    {
        // -- Calibrate the max distance (assume we boot up with nothing
        //    in front of the display)
        uint16_t acc = 0;
        for (int i = 0; i < 4; i++) {
            acc += readIR();
            delay(20);
        }

        m_ir_max = acc/4;
        m_ir_min = 140;

        for (int i = 0; i < 8; i++) {
            m_ir_history[i] = 0;
        }
        Serial.print(m_ir_channel); Serial.print("  "); Serial.println(m_ir_max);
    }
    
    uint8_t senseIR()
    {
        uint16_t raw = readIR();
	    uint16_t drop = m_ir_history[m_ir_cur];

	    // -- Compute a running average by dropping the oldest value and
	    //    adding in the newest value
	    m_ir_running_sum -= drop;
	    m_ir_running_sum += raw;
	
        m_ir_history[m_ir_cur] = raw;
        m_ir_cur++;
        if (m_ir_cur >= 8) m_ir_cur = 0;
	    
        uint16_t ave_ir = m_ir_running_sum >> 3;

        // -- Make sure the value makes sense
        if (ave_ir < m_ir_min) ave_ir = m_ir_min;
        if (ave_ir > m_ir_max) ave_ir = m_ir_max;

        // -- Map into the range 0-255
        uint8_t level = map(ave_ir, m_ir_min, m_ir_max, 0, 255);
        return level;
    }

    uint8_t decayIR(uint8_t cur_level, uint32_t delta_t)
    {
        if (cur_level < m_level) {
            uint8_t new_level = qsub8(m_level, m_down_decay);
            if (cur_level < new_level) {
                m_level = new_level;
            } else {
                m_level = cur_level;
            }
        } else {
            if (cur_level > m_level) {
                uint8_t new_level = qadd8(m_level, m_up_decay);
                if (cur_level > new_level) {
                    m_level = new_level;
                } else {
                    m_level = cur_level;
                }
            }
        }
        return m_level;
    }

    // -------- Pattern management --------

    void replacePattern(Pattern * pattern)
    {
        if (m_pattern)
            delete m_pattern;
            
        m_pattern = pattern;
        m_pattern->setCell(this);
        m_pattern->init();
    }
    
    void clonePattern(Pattern * prototype)
    {
        replacePattern(prototype->clone());
    }
    
    // -------- Visual Rendering --------
    
    void render(uint32_t delta_t)
    {
        if (m_fade_by > 0) {
            fadeToBlackBy(m_fade_by);
        }

        uint8_t level = senseIR();
        if (m_decay) {
            level = decayIR(level, delta_t);
        }
        
        if (m_pattern)
            m_pattern->render(level, delta_t);
    }
};

// -- Global parameters to control IR decay
bool      Cell::m_decay = false;
uint8_t   Cell::m_up_decay = 10;
uint8_t   Cell::m_down_decay = 20;

// ===== 2D Image view ===============================================

#define HUE_BLACK 0xFF

class ImageView
{
private:

    uint8_t  m_Image[IMAGE_WIDTH][IMAGE_HEIGHT];
    uint16_t m_trans_x;
    uint16_t m_trans_y;

    // -- Exact location of each LED
    //    A ring is defined as a 2x2 grid, so each location is a number
    //    between 0 and 4, represented as a 16-bit fixed-point value
    uint16_t m_pixel_x[LEDS_PER_CELL];
    uint16_t m_pixel_y[LEDS_PER_CELL];

public:
    ImageView()
        : m_trans_x(0),
          m_trans_y(0)
    {
        buildmap();
        clearImage();
    }

    uint16_t float_to_fixed(float v)
    {
        uint8_t whole = (uint8_t) v;
        uint8_t frac = (uint8_t) (256.0 * (v - ((float)whole)));
        uint16_t result = whole;
        result <<= 8;
        result |= frac;
        return result;
    }
    
    void buildmap()
    {
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            // -- Divide the ring into 12 equal angles
            float frac = ((float) i) / ((float) LEDS_PER_CELL);
            float angle = frac * 3.1415926535897 * 2.0;
            
            // -- X and Y along a ring of diameter 4
            //    These values will serve as the top left corner of a box
            //    that samples the underlying image.
            float x = (cos(angle) + 1.0) * 2.0;
            float y = (sin(angle) + 1.0) * 2.0;
    
            // -- Convert to fixed point values
            m_pixel_x[i] = float_to_fixed(x);
            m_pixel_y[i] = float_to_fixed(y);
	}
    }
    
    void clearImage()
    {
       for (int i = 0; i < IMAGE_WIDTH; i++) {
            for (int j = 0; j < IMAGE_HEIGHT; j++) {
                m_Image[i][j] = HUE_BLACK;
            }
        }
    }
    
    CRGB getHueFromImage(int x, int y, CRGBPalette16& palette)
    {
        if (x < 0 || x >= IMAGE_WIDTH)  return CRGB::Black;
        if (y < 0 || y >= IMAGE_HEIGHT) return CRGB::Black;
        uint8_t hue = m_Image[x][y];
        if (hue == HUE_BLACK) return CRGB::Black;
        else return ColorFromPalette(palette, hue);
    }
    
    CRGB getImagePixel(int led_index, int ring_x, int ring_y, CRGBPalette16 palette)
    {
        // -- Lookup the relative oordinates of this LED
        //    within its own ring
        uint16_t x_local = m_pixel_x[led_index];
        uint16_t y_local = m_pixel_y[led_index];
        
        // -- Compute its global position using the center of the ring
        int center_x = ring_x * IMAGE_WIDTH_SCALE;
        int center_y = ring_y * IMAGE_HEIGHT_SCALE;
        uint16_t x = (center_x << 8) + x_local + m_trans_x;
        uint16_t y = (center_y << 8) + y_local + m_trans_y;
        
        // -- Get the coordinates of the top left pixel
        uint16_t x_whole = x >> 8;
        uint16_t y_whole = y >> 8;
        
        // -- Get the fractional part, scale down to 0-15
        uint8_t x_frac = (x & 0xFF) >> 4;
        uint8_t y_frac = (y & 0xFF) >> 4;
        
        // -- Sample up to four pixels

        // -- Top left
        CRGB color = getHueFromImage(x_whole, y_whole, palette);
        uint8_t frac_TL = ((0x10 - x_frac) * (0x10 - y_frac)) - 1;
        color.nscale8_video(frac_TL);
        
        if (x_frac > 0) {
            // -- LED crosses into the adjacent pixel to the right
            CRGB color_TR = getHueFromImage(x_whole+1, y_whole, palette);
            uint8_t frac_TR = x_frac * (0x10 - y_frac);
            color_TR.nscale8_video(frac_TR);
            color += color_TR;
        }
        
        if (y_frac > 0) {
            // -- LED crosses into the adjacent pixel below
            CRGB color_BL = getHueFromImage(x_whole, y_whole+1, palette);
            uint8_t frac_BL = (0x10 - x_frac) * y_frac;
            color_BL.nscale8_video(frac_BL);
            color += color_BL;
        }
        
        if (x_frac > 0 and y_frac > 0) {
            // -- LED crosses both boundaries into the corner LED
            CRGB color_BR = getHueFromImage(x_whole+1, y_whole+1, palette);
            uint8_t frac_BR = x_frac * y_frac;
            color_BR.nscale8_video(frac_BR);
            color += color_BR;
        }
        
        return color;
    }

    /** Cubic Hermite for discrete space
     *
     *  Compute an interpolated value between B and C at a distance proportional
     *  to t. A and D are neighboring values that affect the curve near the edges
     *  (A is on the other side of B; D is on the other side of C):
     *
     *    A ....... B ....... C ....... D
     *              |<-t----->|
     *                 ^ 
     *                 | interpolated value
     *                 
     *  This version is discretized to make it more efficient: it can only
     *  interpolate 9 points in between. In other words, t should be a value
     *  in the range [0,8].
     *
     *  This choice makes certain computations fast. For example, instead of
     *  computing a * t * t * t (for values of t in [0,1]), we can compute
     *      a * (t/8) * (t/8) * (t/8)
     *    = a * t * t * t / (8 * 8 * 8)
     *    = (a * t * t * t) >> 9
     */

    uint8_t cubic8( uint8_t A, uint8_t B, uint8_t C, uint8_t D, uint16_t t)
    {
        uint32_t a = (D + 3*B - 3*C - A)/2;
        uint32_t b = A + 2*C - (5*B + D)/2;
        uint32_t c = (C - A)/2;
        uint32_t d = B;

        // uint32_t a = -Ao2 + (3*B)/2 - (3*C)/2 + Do2;
        // uint32_t b = A - (5*B)/2 + 2*C - Do2;
        // uint32_t c = -Ao2 + C/2;

        uint32_t at3 = (a * t * t * t) >> 9;
        uint32_t bt2 = (b * t * t) >> 6;
        uint32_t ct  = (c * t) >> 3;
        uint32_t r32 = at3 + bt2 + ct + d;
        return uint8_t(r32);
    }
};

// ===== Patterns ====================================================

class SolidPattern : public Pattern
{
public:
    SolidPattern()
        : Pattern()
    {}

    virtual Pattern * clone()
    {
        return new SolidPattern(*this);
    }

    virtual void render(uint8_t level, uint32_t delta_t)
    {
        if (level > 230) level = 230;
        m_cell->setAllLEDs(level);
    }
};

class DotPattern : public Pattern
{
private:
    uint16_t m_min_speed;
    uint16_t m_max_speed;
    uint16_t m_position;

public:
    DotPattern(int min_speed, int max_speed)
        : Pattern(),
          m_min_speed(min_speed << 8),
          m_max_speed(max_speed << 8),
          m_position(0)
    {}

    virtual Pattern * clone()
    {
        return new DotPattern(*this);
    }

    void init()
    {
        m_cell->setFadeBy(20);
        m_position = random16();
    }

    virtual void render(uint8_t level, uint32_t delta_t)
    {
        if (level > 230) level = 230;
        int speed = map(level, 0, 255, m_max_speed, m_min_speed);
        m_position += speed;
        m_cell->setPixel(m_position, level);
    }
};

class FirePattern : public Pattern
{ 
private:
    int m_sparking;
    byte m_heat[LEDS_PER_CELL];

public:
    FirePattern(int sparking)
        : Pattern(),
          m_sparking(sparking)
    {}

    virtual Pattern * clone()
    {
        return new FirePattern(*this);
    }

    virtual void init()
    {
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            m_heat[i] = 0;
        }
        m_cell->setPalette(HeatColors_p);
        m_cell->setTranslation(random16());
    }

    void render(uint8_t level, uint32_t delta_t)
    { 
        int num_leds = LEDS_PER_CELL;
      
        // Temperature is in arbitrary units from 0 (cold black) to 255 (white hot).
      
        // COOLING: How much does the air cool as it rises?
        // Less cooling = taller flames.  More cooling = shorter flames.
        // Default 55, suggested range 20-100       
        int cooling = map(level, 0, 255, 20, 150);
        
        // Step 1.  Cool down every cell a little
        for( int i = 0; i < num_leds; i++) {
            m_heat[i] = qsub8( m_heat[i],  random8(0, ((cooling * 10) / num_leds) + 2));
        }
    
        // Step 2.  Heat from each cell drifts 'up' and diffuses a little
        for( int k= num_leds - 1; k >= 2; k--) {
            m_heat[k] = (m_heat[k - 1] + m_heat[k - 2] + m_heat[k - 2] ) / 3;
        }
      
        // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
        if( random8() < m_sparking ) {
            int y = random8(3);
            m_heat[y] = qadd8( m_heat[y], random8(100,150) );
        }
  
        // Step 4.  Map from heat cells to LED colors
        for( int j = 0; j < num_leds; j++) {
            // Scale the heat value from 0-255 down to 0-240
            // for best results with color palettes.
            byte colorindex = scale8( m_heat[num_leds - j - 1], 170);
            m_cell->setLED(j, colorindex);
        }
    }
};

class WavePattern : public Pattern
{
public:

    WavePattern()
        : Pattern()
    {}

    virtual Pattern * clone()
    {
        return new WavePattern(*this);
    }

    // ColorWavesWithPalettes by Mark Kriegsman:
    // https://gist.github.com/kriegsman/8281905786e8b2632aeb
    // This function draws color waves with an ever-changing,
    // widely-varying set of parameters, using a color palette.
    void render(uint8_t level, uint32_t delta_t)
    {
        static uint16_t sPseudotime = 0;
        static uint16_t sLastMillis = 0;
        static uint16_t sHue16 = 0;
    
        // uint8_t sat8 = beatsin88( 87, 220, 250);
        uint8_t brightdepth = beatsin88( 341, 96, 224);
        uint16_t brightnessthetainc16 = beatsin88( 203, (25 * 256), (40 * 256));
        uint8_t msmultiplier = beatsin88(147, 23, (64 - (level/4)) + 60);
    
        uint16_t hue16 = sHue16;//gHue * 256;
        uint16_t hueinc16 = beatsin88(113, 300, 1500);
    
        uint16_t ms = millis();
        uint16_t deltams = ms - sLastMillis ;
        sLastMillis  = ms;
        sPseudotime += deltams * msmultiplier;
        sHue16 += deltams * beatsin88( 400, 5, 9);
        uint16_t brightnesstheta16 = sPseudotime;
    
        for ( uint16_t i = 0 ; i < LEDS_PER_CELL; i++) {
            hue16 += hueinc16;
            uint8_t hue8 = hue16 / 256;
            uint16_t h16_128 = hue16 >> 7;
            if ( h16_128 & 0x100) {
                hue8 = 255 - (h16_128 >> 1);
            } else {
                hue8 = h16_128 >> 1;
            }
    
            brightnesstheta16  += brightnessthetainc16;
            uint16_t b16 = sin16( brightnesstheta16  ) + 32768;
    
            uint16_t bri16 = (uint32_t)((uint32_t)b16 * (uint32_t)b16) / 65536;
            uint8_t bri8 = (uint32_t)(((uint32_t)bri16) * brightdepth) / 65536;
            bri8 += (255 - brightdepth);
    
            uint8_t index = hue8;
            //index = triwave8( index);
            index = scale8( index, 240);

            m_cell->setLED(i, index, bri8);
            /*
              CRGB newcolor = ColorFromPalette(m_palette, index, bri8);
              nblend(getLED(i), newcolor, 128);
            */
        }
    }
};

class DiscoPattern : public Pattern
{
private:
    uint8_t m_range;
    uint8_t m_speed;
    
    uint8_t hue[LEDS_PER_CELL];
    uint8_t hue_level[LEDS_PER_CELL];
    uint8_t position[LEDS_PER_CELL];

public:
    DiscoPattern(int range, int speed)
        : Pattern(),
          m_range(range),
          m_speed(speed)
    {}

    virtual Pattern * clone()
    {
        return new DiscoPattern(m_range, m_speed);
    }

    void init()
    {
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            // -- Random hues
            hue[i] = random8(m_range);
            hue_level[i] = 0;
            // -- Random starting points on the curve
            position[i] = random8();
        }        
    }

    void render(uint8_t level, uint32_t delta_t) 
    {
        uint8_t adjusted_level = lerp8by8(0, 240, level);
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            // -- Compute brightness a position on cosine wave
            uint8_t prev_pos = position[i];
            position[i] = prev_pos + m_speed; // Should give about 1/2 second per cycle
            uint8_t cur_pos = position[i];
            uint8_t wave = cos8(cur_pos);
            uint8_t brightness = lerp8by8(20, 255, wave);

            // -- Hue is incremented to match the level
            uint8_t cur_hue_level = hue_level[i];
            if (cur_hue_level < adjusted_level) hue_level[i] += min(2, adjusted_level - cur_hue_level);
            if (cur_hue_level > adjusted_level) hue_level[i] = (cur_hue_level * 3 + adjusted_level) / 4;
        
            m_cell->setLED(i, hue[i] + hue_level[i], brightness);

            // -- At the end of each cycle, pick a new color
            if (prev_pos < 128 && cur_pos >= 128) {
                hue[i] = random8(m_range);
            }
        }
    }
};

class ImagePattern : public Pattern
{
private:
    ImageView * m_image;

public:
    ImagePattern(ImageView * image)
        : Pattern(),
          m_image(image)
    {}

    virtual Pattern * clone()
    {
        return new ImagePattern(m_image);
    }

    void render(uint8_t level, uint32_t delta_t) 
    {
        // -- For each LED in this ring, sample the colors
        //    from the backing image to get a final color
        for (int i = 0; i < LEDS_PER_CELL; i++) {
            CRGB color = m_image->getImagePixel(i, m_cell->getX(), m_cell->getY(), m_cell->getPalette());
            m_cell->setRawLED(i, color);
        }
    }
};

// ===== Table ====================================================

class ReacTable
{
private:

    Cell * m_Cells[NUM_CELLS];

public:

    ReacTable() 
    {
        for (int i = 0; i < NUM_CELLS; i++) {
            m_Cells[i] = 0;
        }
    }

    void init()
    {
        int index = 0;
        //                         IRin IRch X   Y  LED
        m_Cells[index++] = new Cell( 0,  0,  0,  0, 60 );
        m_Cells[index++] = new Cell( 0,  1,  2,  0, 49 );
        m_Cells[index++] = new Cell( 0,  2,  4,  0, 38 );
        m_Cells[index++] = new Cell( 0,  3,  6,  0, 27 );
        m_Cells[index++] = new Cell( 0,  4,  8,  0, 16 );
        m_Cells[index++] = new Cell( 0,  5, 10,  0,  5 );
        m_Cells[index++] = new Cell( 0,  6,  1,  1, 54 );
        m_Cells[index++] = new Cell( 0,  7,  3,  1, 43 );
        m_Cells[index++] = new Cell( 0,  8,  5,  1, 32 );
        m_Cells[index++] = new Cell( 0,  9,  7,  1, 21 );
        m_Cells[index++] = new Cell( 0, 10,  9,  1, 10 );
        m_Cells[index++] = new Cell( 0, 11,  0,  2, 59 );
        m_Cells[index++] = new Cell( 0, 12,  2,  2, 48 );
        m_Cells[index++] = new Cell( 0, 13,  4,  2, 37 );
        m_Cells[index++] = new Cell( 0, 14,  6,  2, 26 );
        m_Cells[index++] = new Cell( 0, 15,  8,  2, 15 );
        m_Cells[index++] = new Cell( 1,  0, 10,  2,  4 );
        m_Cells[index++] = new Cell( 1,  1,  1,  3, 53 );
        m_Cells[index++] = new Cell( 1,  2,  3,  3, 42 );
        m_Cells[index++] = new Cell( 1,  3,  5,  3, 31 );
        m_Cells[index++] = new Cell( 1,  4,  7,  3, 20 );
        m_Cells[index++] = new Cell( 1,  5,  9,  3,  9 );
        m_Cells[index++] = new Cell( 1,  6,  0,  4, 58 );
        m_Cells[index++] = new Cell( 1,  7,  2,  4, 47 );
        m_Cells[index++] = new Cell( 1,  8,  4,  4, 36 );
        m_Cells[index++] = new Cell( 1,  9,  6,  4, 25 );
        m_Cells[index++] = new Cell( 1, 10,  8,  4, 14 );
        m_Cells[index++] = new Cell( 1, 11, 10,  4,  3 );
        m_Cells[index++] = new Cell( 1, 12,  1,  5, 52 );
        m_Cells[index++] = new Cell( 1, 13,  3,  5, 41 );
        m_Cells[index++] = new Cell( 1, 14,  5,  5, 30 );
        m_Cells[index++] = new Cell( 1, 15,  7,  5, 19 );
        m_Cells[index++] = new Cell( 2,  0,  9,  5,  8 );
        m_Cells[index++] = new Cell( 2,  1,  0,  6, 57 );
        m_Cells[index++] = new Cell( 2,  2,  2,  6, 46 );
        m_Cells[index++] = new Cell( 2,  3,  4,  6, 35 );
        m_Cells[index++] = new Cell( 2,  4,  6,  6, 24 );
        m_Cells[index++] = new Cell( 2,  5,  8,  6, 13 );
        m_Cells[index++] = new Cell( 2,  6, 10,  6,  2 );
        m_Cells[index++] = new Cell( 2,  7,  1,  7, 51 );
        m_Cells[index++] = new Cell( 2,  8,  3,  7, 40 );
        m_Cells[index++] = new Cell( 2,  9,  5,  7, 29 );
        m_Cells[index++] = new Cell( 2, 10,  7,  7, 18 );
        m_Cells[index++] = new Cell( 2, 11,  9,  7,  7 );
        m_Cells[index++] = new Cell( 2, 12,  0,  8, 56 );
        m_Cells[index++] = new Cell( 2, 13,  2,  8, 45 );
        m_Cells[index++] = new Cell( 2, 14,  4,  8, 34 );
        m_Cells[index++] = new Cell( 2, 15,  6,  8, 23 );
        m_Cells[index++] = new Cell( 3,  0,  8,  8, 12 );
        m_Cells[index++] = new Cell( 3,  1, 10,  8,  1 );
        m_Cells[index++] = new Cell( 3,  2,  1,  9, 50 );
        m_Cells[index++] = new Cell( 3,  3,  3,  9, 39 );
        m_Cells[index++] = new Cell( 3,  4,  5,  9, 28 );
        m_Cells[index++] = new Cell( 3,  5,  7,  9, 17 );
        m_Cells[index++] = new Cell( 3,  6,  9,  9,  6 );
        m_Cells[index++] = new Cell( 3,  7,  0, 10, 55 );
        m_Cells[index++] = new Cell( 3,  8,  2, 10, 44 );
        m_Cells[index++] = new Cell( 3,  9,  4, 10, 33 );
        m_Cells[index++] = new Cell( 3, 10,  6, 10, 22 );
        m_Cells[index++] = new Cell( 3, 11,  8, 10, 11 );
        m_Cells[index++] = new Cell( 3, 12, 10, 10,  0 );
        assert( index == NUM_CELLS);
    }

    void setPattern(Pattern * prototype) {
        for (int i = 0; i < NUM_CELLS; i++) {
            m_Cells[i]->clonePattern( prototype );
        }
    }
    
    void render(uint32_t delta_t)
    {
        for (int i = 0; i < NUM_CELLS; i++) {
            m_Cells[i]->render(delta_t);
        }
    }
};

/*
typedef CellWrapper<Fade<40,
                         ColorDot<400, 4000, 
                                  DecayLevel< 2, 12,
                                              IRSense< BasePattern > > > > > Cell1;

typedef CellWrapper<Fire< 100, 
                          //DecayLevel< 1, 16,
                          IRSense< BasePattern > > > Cell2;

typedef CellWrapper< Fade<40, 
                          ColorSolid< 
                              DecayLevel<1, 4, 
                                         IRSense< BasePattern > > > > > Cell3;

typedef CellWrapper< ColorSolid< DecayLevel<4, 10, IRSense< BasePattern > > > > Cell4;

typedef CellWrapper< Disco<60, 3, 
                           DecayLevel<1, 4,
                                      IRSense< BasePattern > > > > Cell5;


Cell * m_Cells[NUM_CELLS];
*/


// ===== Buttons and controls =============

adel waitTurn(int pinA, int pinB)
{
 abegin:
    while (1) {
        if ( digitalRead(pinA) == HIGH) {
            await( digitalRead(pinA) == LOW  );
            if ( digitalRead(pinB) == HIGH) {
                await( digitalRead(pinB) == LOW  );
                // if ( digitalRead(pinA) == LOW)
                afinish;
            }
        }
        adelay(1);
    }

    aend;
}

adel rotary(int * pDelta)
{
 abegin:
    (*pDelta) = 0;
    auntil( waitTurn(ROTARY_A_PIN, ROTARY_B_PIN), 
            waitTurn(ROTARY_B_PIN, ROTARY_A_PIN) ) {
        (*pDelta) = 1;
    } else {
        (*pDelta) = -1;
    }
    aend;
}
  
adel waitbutton(int pin)
{
 abegin:
    while (1) {
        await (digitalRead(pin) == HIGH);
        adelay (50);
        if (digitalRead(pin) == HIGH) {
            await (digitalRead(pin) == LOW);
            afinish;
        }
    }
    aend;
}

adel adjustments()
{
    int delta;
    int val;
    
 abegin:
    val = 0;
    
    while (1) {
        andthen( rotary( &delta ) );
        val += delta;
        // Serial.print("Val = ");
        // Serial.println(val);
    }

    aend;
}

adel changemode()
{
 abegin:
    while (1) {
        andthen( waitbutton (BUTTON_PIN) );
        Serial.println("Button");
    }
    aend;
}

// ===== Main Routines ============================

// -- Global table object
ReacTable g_Table;

// -- Global image object
ImageView g_Image;

void setup()
{
    pinMode(LED_PIN, OUTPUT);

    pinMode(BUTTON_PIN, INPUT);
    pinMode(ROTARY_A_PIN, INPUT_PULLUP);
    pinMode(ROTARY_B_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    delay(200);

    pinMode(MIC_PIN, INPUT);
    
    // -- Set up the LED rings (as a giant strip)
    FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(g_LEDs, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(g_Brightness);
    
    fill_solid(g_LEDs, NUM_LEDS, CRGB::Black);
    FastLED.show();
    
    // -- Set up the OLED display
    
    display.init();
    
    // display.flipScreenVertically();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);

    for (int i = 0; i < 4; i++) {
        pinMode(IR_INPUT[i], INPUT);
    }

    pinMode(IR_CHANNEL_BIT_0, OUTPUT);
    pinMode(IR_CHANNEL_BIT_1, OUTPUT);
    pinMode(IR_CHANNEL_BIT_2, OUTPUT);
    pinMode(IR_CHANNEL_BIT_3, OUTPUT);

    g_Table.init();
    
    SolidPattern solid;
    FirePattern fire(150);
    DiscoPattern disco(60,3);
    DotPattern dot(10,40);
    g_Table.setPattern(&dot);
    
    // Serial.println("READY!");
    display.drawString(10, 10, "Ready!");
    display.display();
    delay(2000);

    // uint32_t freemem = esp_get_free_heap_size();
    // Serial.print("Free memory: ");
    // Serial.println(freemem);
}

adel render()
{
    uint32_t delta_time = 0;
    uint32_t last_time = 0;
    
 abegin:
    while (1) {
        delta_time = millis() - last_time;
        last_time = millis();
        g_Table.render(delta_time);
        FastLED.show();
        if (1) {
            int val = analogRead(MIC_PIN);
            int scaled = map(val, 0, 4000, 0, 100);
            display.clear();
            display.fillRect(10, 30, scaled, 10);
            display.display();
        }
        adelay(1000/FRAMES_PER_SECOND);
    }
    aend;
}

/*
void draw_image()
{
    int center_x = IMAGE_WIDTH/2;
    int center_y = IMAGE_HEIGHT/2;
    for (int i = 0; i < 20; i++) {
       g_Image[center_x + i][center_y] = 50;
       g_Image[center_x + i][center_y+1] = 50;
       g_Image[center_x + i][center_y+9] = 130;
       g_Image[center_x + i][center_y+10] = 130;
       g_Image[center_x + i][center_y+19] = 50;
       g_Image[center_x + i][center_y+20] = 50;
    }
    for (int j = 0; j < 20; j++) {
       g_Image[center_x][center_y+j] = 50;
       g_Image[center_x+1][center_y+j] = 50;
       g_Image[center_x+9][center_y+j] = 130;
       g_Image[center_x+10][center_y+j] = 130;
       g_Image[center_x+19][center_y+j] = 50;
       g_Image[center_x+20][center_y+j] = 50;
    }
}

void computeIRCenter(uint16_t & cx, uint16_t & cy)
{
    uint32_t total_x = 0;
    uint32_t total_y = 0;
    uint32_t count = 0;
    for (int i = 0; i < NUM_CELLS; i++) {
        uint8_t level = m_Cells[i]->senseIR();
        if (level < 128) {
            uint32_t scale = (128 - level)/32;
            total_x += scale * m_Cells[i]->getX();
            total_y += scale * m_Cells[i]->getY();
            count += scale;
        }
    }

    if (count == 0) {
        cx = IMAGE_WIDTH/2;
        cy = IMAGE_WIDTH/2;
    } else {
        cx = (total_x / count) * 5;
        cy = (total_y / count) * 3;
        Serial.print(cx); Serial.print(" ");
        Serial.println(cy);
    }
}

adel do_image()
{
    uint16_t cx;
    uint16_t cy;
    abegin:
    draw_image();
    while (1) {
        computeIRCenter(cx, cy);
        g_trans_x = (-cx) << 8;
        g_trans_y = (-cy) << 8;
        renderImage();
        FastLED.show();
        
        g_trans_x += 0x100;
        if (g_trans_x > 0x4000) {
            g_trans_x = 0;
            g_trans_y = 0;
        }
        g_trans_y += 0x70;
        
        adelay(1000/FRAMES_PER_SECOND);
    }
    aend;
}
*/

void loop()
{
    // arepeat( adjustments() );
    // arepeat( changemode() );
    arepeat( render() );
    // arepeat( do_image() );
}

