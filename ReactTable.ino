#include <adel.h>
#include "SSD1306Wire.h"
#include "SH1106.h"
#define FASTLED_RMT_BUILTIN_DRIVER true
#include <FastLED.h>

// ===== Global configuration =====================================

#define ROTARY_A_PIN 19
#define ROTARY_B_PIN 18
#define BUTTON_PIN 32

#define NUM_IR_CHANNELS 16
#define IR_CHANNEL_BIT_0 0 
// 2  // 12
#define IR_CHANNEL_BIT_1 4 
// 0  // 14
#define IR_CHANNEL_BIT_2 16 
// 4  // 27
#define IR_CHANNEL_BIT_3 17 
// 16 // 26

#define NUM_IR_INPUTS 4
int IR_INPUT_PIN[] = { 27, 14, 12, 13 }; // { 35, 34, 39, 36 };

#define SDA_PIN 21
#define SCL_PIN 22

#define X_PIN 32
#define Y_PIN 33
#define Z_PIN 25

#define MIC_PIN 13

#define LED_PIN 17
#define LED_PIN_1 2 // 13
#define LED_PIN_2 5 // 14
#define LED_PIN_3 15 // 27

#define NUM_CELLS 61
#define LEDS_PER_CELL 12
#define NUM_LEDS (NUM_CELLS * LEDS_PER_CELL)

#define NUM_LEDS_1 (22 * LEDS_PER_CELL)
#define NUM_LEDS_2 (22 * LEDS_PER_CELL)
#define NUM_LEDS_3 (17 * LEDS_PER_CELL)

#define FRAMES_PER_SECOND 30

#define CELL_COLS 12
#define CELL_ROWS 12

#define IMAGE_WIDTH_SCALE  5
#define IMAGE_HEIGHT_SCALE 3
#define IMAGE_WIDTH  ((CELL_COLS+2) * IMAGE_WIDTH_SCALE)
#define IMAGE_HEIGHT ((CELL_ROWS+2) * IMAGE_HEIGHT_SCALE)

// -- The core to run FastLED.show()
#define FASTLED_SHOW_CORE 0

// ===== LED strip information ====================================

CRGB g_LEDs[NUM_LEDS + 10];

uint8_t g_Brightness = 64;

#define COLOR_ORDER GRB
#define CHIPSET     WS2812

// ===== FastLED Show task ========================================

bool gShowOnOtherCore = true;

// -- Task handles for use in the notifications
static TaskHandle_t FastLEDshowTaskHandle = 0;
static TaskHandle_t userTaskHandle = 0;

/** show() for ESP32
    Call this function instead of FastLED.show(). It signals core 0 to issue a show,
    then waits for a notification that it is done.
*/
void FastLEDshowESP32()
{
  if ( ! gShowOnOtherCore) {
    FastLED.show();
  } else {
    if (userTaskHandle == 0) {
      //const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );
      // -- Store the handle of the current task, so that the show task can
      //    notify it when it's done
      userTaskHandle = xTaskGetCurrentTaskHandle();

      // -- Trigger the show task
      xTaskNotifyGive(FastLEDshowTaskHandle);

      // -- Wait to be notified that it's done
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      userTaskHandle = 0;
    }
  }
}

/** show Task
    This function runs on core 0 and just waits for requests to call FastLED.show()
*/
void FastLEDshowTask(void *pvParameters)
{
  //const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 500 );
  // -- Run forever...
  for (;;) {
    // -- Wait for the trigger
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // -- Do the show (synchronously)
    uint32_t start = micros();
    FastLED.show();
    uint32_t finish = micros();

    // Serial.println(finish-start);

    // -- Notify the calling task
    xTaskNotifyGive(userTaskHandle);
  }
}

// ===== Cells ====================================================

/** Cells

    Each cell represents one IR sensor and an LED ring. The Cell class
    is in charge of holding information about the LEDs and includes a bunch
    of useful methods for drawing on them. These methods are called by the
    subsequent layers in the mixin stack.
*/

class Cell
{
  protected:
    // -- Cell position
    int             m_x;
    int             m_y;
    int             m_index;

    // -- LED ring information
    uint16_t        m_led_index;
    uint16_t        m_translate;
    CRGBPalette16   m_palette;

    // -- IR sensor
    int             m_ir_input;
    int             m_ir_channel;
    uint16_t        m_ir_min;
    uint16_t        m_ir_max;
    uint8_t         m_ir_channel_selector[4];

  public:
    Cell(int ir_input, int ir_channel, int x, int y, int index )
      : m_x(x),
        m_y(y),
        m_index(index),
        m_led_index(index * LEDS_PER_CELL),
        m_translate(0),
        m_palette(RainbowColors_p),
        m_ir_input(ir_input),
        m_ir_channel(ir_channel),
        m_ir_min(0),
        m_ir_max(0)
    {
      m_ir_channel_selector[0] = (m_ir_channel & 0x1) ? HIGH : LOW;
      m_ir_channel_selector[1] = (m_ir_channel & 0x2) ? HIGH : LOW;
      m_ir_channel_selector[2] = (m_ir_channel & 0x4) ? HIGH : LOW;
      m_ir_channel_selector[3] = (m_ir_channel & 0x8) ? HIGH : LOW;
    }

    // ----- Getters and setters -----

    // -- Set the palette explicitly
    void setPalette(CRGBPalette16 new_palette) {
      m_palette = new_palette;
    }

    CRGBPalette16 getPalette() const {
      return m_palette;
    }

    void setTranslation(uint16_t trans) {
      m_translate = trans;
    }
    uint16_t getTranslation(void) const {
      return m_translate;
    }

    uint16_t getX() const {
      return m_x;
    }
    uint16_t getY() const {
      return m_y;
    }

    // ----- IR Sensors -----

    /** Set the IR max and min
        Determined by the calibration phase.
    */
    void setIRMax(uint16_t ir_max) {
      m_ir_max = ir_max;
    }

    void setIRMin(uint16_t ir_min) {
      m_ir_min = ir_min;
    }

    /** Read raw IR value
        We can have several MUXs, each with 16 channels. To read a specific
        IR value, we specify which MUX (the "input") and which channel.
    */
    uint16_t rawIR()
    {
      // -- Select the channel
      digitalWrite(IR_CHANNEL_BIT_0, m_ir_channel_selector[0]);
      digitalWrite(IR_CHANNEL_BIT_1, m_ir_channel_selector[1]);
      digitalWrite(IR_CHANNEL_BIT_2, m_ir_channel_selector[2]);
      digitalWrite(IR_CHANNEL_BIT_3, m_ir_channel_selector[3]);

      // -- Finally, read the analog value
      uint16_t val;
      val = analogRead(IR_INPUT_PIN[m_ir_input]);
      val = analogRead(IR_INPUT_PIN[m_ir_input]);
      return val;
    }

    /** Read
        Read and map to the calibrated range
    */
    uint8_t senseIR()
    {
      uint16_t val = rawIR();

      // -- Pin the value in between the min and max from calibration
      if (val < m_ir_min) val = m_ir_min;
      if (val > m_ir_max) val = m_ir_max;

      // -- Map to 8-bit value
      uint8_t level = map(val, m_ir_min, m_ir_max, 0, 255);
      return level;
    }

    // ----- Rendering methods -----

    /* Get and Set raw LEDs
       Access the ring without any translation or interpolation.
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

       The position is expressed as a 16-bit fixed-precision number that maps
       to whatever number of actual LEDs there are in the strip/ring. This
       function dithers the adjacent pixels to create a smooth transition.
       It needs to be fast!
    */
    void setPixel(uint16_t pos_in, CRGB color) {
      // -- Apply translation
      //    This might overflow, which supports LED rings
      uint16_t pos = pos_in + m_translate;

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

    /** Set a pixel
        This version just chooses the color from the palette.
    */
    void setPixel(uint16_t pos, uint8_t hue, uint8_t brightness = 255) {
      // -- Get the color from the palette
      CRGB color = ColorFromPalette(m_palette, hue);
      if (brightness < 255) color.nscale8_video(brightness);
      setPixel(pos, color);
    }

    /** Set an LED

        Set an LED value based on LED index rather than abstract position.
        Other than possible translation of coordinates, this should end
        up setting the color on a particular real LED device.
    */
    void setLED(uint8_t led, uint8_t hue, uint8_t brightness = 255) {
      uint16_t pos = ((((uint16_t) led) << 8) / LEDS_PER_CELL) << 8;
      setPixel(pos, hue, brightness);
    }

    /** Set all LEDs

        Set all the LEDs to a particular color. Since they are all the
        same, there is no need to use set_pixel.
    */
    void setAllLEDs(CRGB& color) {
      for (int i = 0; i < LEDS_PER_CELL; i++) {
        g_LEDs[m_led_index + i] = color;
      }
    }

    /** Set all LEDs

        Set all the LEDs to a particular hue.
    */
    void setAllLEDs(uint8_t hue, uint8_t brightness = 255) {
      CRGB color = ColorFromPalette(m_palette, hue);
      if (brightness < 255) color.nscale8_video(brightness);
      setAllLEDs(color);
    }

    /** Fade all LEDs

    */
    void fadeToBlackBy(uint8_t howmuch) {
      for (int i = 0; i < LEDS_PER_CELL; i++) {
        g_LEDs[m_led_index + i].fadeToBlackBy(howmuch);
      }
    }

};

// ===== Base layer ===============================================

/** Base class for all patterns
    Provides the "bottom" for the mixin of patterns
*/

class BasePattern
{
  protected:
    Cell * m_cell;

    BasePattern(Cell * cell)
      : m_cell(cell)
    {}

  public:
    void init() {}
    uint8_t sense(uint32_t delta_t) {
      return 0;
    }
    void render(uint8_t level, uint32_t delta_t) {}
};

// ===== Sensing layers ===========================================

/** IR sensor

    Return the value of the IR sensor as the "sense" value
*/
template<typename Super>
class IRSense : public Super
{
  public:
    IRSense(Cell * cell)
      : Super(cell)
    {}

    // ----- Mixin methods ----------

    void init()
    {
      Super::init();
    }

    uint8_t sense(uint32_t delta_t)
    {
      return Super::m_cell->senseIR();
    }
};

/** Decay sense value

    Smooth out the sensor value by migrating it up and down according to
    the UP and DOWN rates. Sensor values range from 0 (bottom) to 255
    (top), and UP and DOWN are expresses in units per second. It usually
    looks nice if UP is slow and DOWN is fast.
*/
template<uint16_t UP, uint16_t DOWN, typename Super>
class SenseDecay : public Super
{
  private:
    uint16_t   m_level;

  public:
    SenseDecay(Cell * cell)
      : Super(cell),
        m_level(0)
    {}

    // ----- Mixin methods ----------

    void init() {
      Super::init();
    }

    uint8_t sense(uint32_t delta_t)
    {
      uint8_t cur_level = Super::sense(delta_t);

      if (cur_level < m_level) {
        uint8_t new_level = qsub8(m_level, DOWN);
        if (cur_level < new_level) {
          m_level = new_level;
        } else {
          m_level = cur_level;
        }
      } else {
        if (cur_level > m_level) {
          uint8_t new_level = qadd8(m_level, UP);
          if (cur_level > new_level) {
            m_level = new_level;
          } else {
            m_level = cur_level;
          }
        }
      }
      return m_level;
    }
};

template<uint8_t SPEED, typename Super>
class SenseSineWave : public Super
{
  private:
    uint16_t   m_level;
    
  public:
    SenseSineWave(Cell * cell)
      : Super(cell),
        m_level(0)
    {}

    // ----- Mixin methods ----------

    void init() {
      Super::init();
    }

    uint8_t sense(uint32_t delta_t) {
        m_level += SPEED;
        return sin8(m_level);
    }
};

// ===== Pattern layers ===========================================


/*
    CRGBPalette16 m_palette[7];
    const char *  m_paletteName[7];
    int           m_curPalette;
       m_palette[0] = RainbowColors_p;
        m_paletteName[0] = "Rainbow";

        m_palette[1] = CloudColors_p;
        m_paletteName[1] = "Clouds";

        m_palette[2] = LavaColors_p;
        m_paletteName[2] = "Lava";

        m_palette[3] = OceanColors_p;
        m_paletteName[3] = "Ocean";

        m_palette[4] = ForestColors_p;
        m_paletteName[4] = "Forest";

        m_palette[5] = PartyColors_p;
        m_paletteName[5] = "Party";

        m_palette[6] = HeatColors_p;
        m_paletteName[6] = "Heat";
*/

template<typename Super>
class RainbowColors : public Super
{
  public:

    RainbowColors(Cell * cell)
      : Super(cell)
    {}

    // ----- Mixin methods -----

    void init() {
      Super::init();
      Super::m_cell->setPalette(RainbowColors_p);
    }
};

template<typename Super>
class HeatColors : public Super
{
  public:

    HeatColors(Cell * cell)
      : Super(cell)
    {}

    // ----- Mixin methods -----

    void init() {
      Super::init();
      Super::m_cell->setPalette(HeatColors_p);
    }
};

template<typename Super>
class PartyColors : public Super
{
  public:

    PartyColors(Cell * cell)
      : Super(cell)
    {}

    // ----- Mixin methods -----

    void init() {
      Super::init();
      Super::m_cell->setPalette(PartyColors_p);
    }
};


template<int FADE_BY, typename Super>
class Fade : public Super
{
  public:

    Fade(Cell * cell)
      : Super(cell)
    {}

    // ----- Mixin methods -----

    void init() {
      Super::init();
    }

    void render(uint8_t level, uint32_t delta_t) {
      Super::m_cell->fadeToBlackBy(FADE_BY);
      Super::render(level, delta_t);
    }
};

template<typename Super>
class Clear : public Super
{
  public:

    Clear(Cell * cell)
      : Super(cell)
    {}

    // ----- Mixin methods -----

    void init() {
      Super::init();
    }

    void render(uint8_t level, uint32_t delta_t) {
      CRGB b = CRGB::Black;
      Super::m_cell->setAllLEDs(b);
      Super::render(level, delta_t);
    }
};


template<typename Super>
class ColorSolid : public Super
{
  public:
    ColorSolid(Cell * cell)
      : Super(cell)
    {}

    // ----- Mixin methods ----------

    void init() {
      Super::init();
    }

    void render(uint8_t level, uint32_t delta_t) {
      if (level > 230) level = 230;
      Super::m_cell->setAllLEDs(level);
    }
};

template<int MIN_SPEED, int MAX_SPEED, typename Super>
class ColorDot : public Super
{
  private:

    uint16_t m_position;

  public:

    ColorDot(Cell * cell)
      : Super(cell),
        m_position(0)
    {}

    // ----- Mixin methods -----

    void init() {
      Super::init();
      m_position = random16();
    }

    void render(uint8_t level, uint32_t delta_t) {
      Super::render(level, delta_t);
      int speed = map(level, 0, 255, MAX_SPEED, MIN_SPEED);
      m_position += speed;
      if (level > 230) level = 230;
      Super::m_cell->setPixel(m_position, level);
      Super::m_cell->setPixel(m_position + 0x5555, level);
      Super::m_cell->setPixel(m_position + 0xAAAA, level);
    }
};

template<bool REACT, typename Super>
class Confetti : public Super
{
  public:
    Confetti(Cell * cell)
      : Super(cell)
    {}

    void init() {
      Super::init();
    }

    void render(uint8_t level, uint32_t delta_t) {
      Super::render(level, delta_t);
      uint8_t prob = 128;
      if (REACT) {
        prob = 255 - level;
      }
      uint8_t coin = random8();
      if (coin < prob) {
        int pos = random8(LEDS_PER_CELL);
        Super::m_cell->setLED(pos, level + random8(64));
        // leds[pos] += CHSV( gHue + random8(64), 200, 255);
      }
    }
};

template<bool REACT, uint8_t CHANCE, typename Super>
class Glitter : public Super
{
  public:
    Glitter(Cell * cell)
      : Super(cell)
    {}

    void init() {
      Super::init();
    }

    void render(uint8_t level, uint32_t delta_t) {
      Super::render(level, delta_t);
      uint8_t chance;
      if (REACT) {
        chance = map(level, 0, 255, 0, CHANCE);
      } else {
        chance = CHANCE;
      }
      if (random8() < chance) {
        Super::m_cell->setLED(random16(), CRGB::White);
      }
    }
};

template<typename Super>
class Fire : public Super
{
  private:
    int m_sparking;
    byte m_heat[LEDS_PER_CELL];

  public:
    Fire(Cell * cell)
      : Super(cell),
        m_sparking(150)
    {}

    void init()
    {
      Super::init();
      for (int i = 0; i < LEDS_PER_CELL; i++) {
        m_heat[i] = 0;
      }
      Super::m_cell->setTranslation(random16());
    }

    void render(uint8_t level, uint32_t delta_t)
    {
      Super::render(level, delta_t);

      int num_leds = LEDS_PER_CELL;

      // Temperature is in arbitrary units from 0 (cold black) to 255 (white hot).

      // COOLING: How much does the air cool as it rises?
      // Less cooling = taller flames.  More cooling = shorter flames.
      // Default 55, suggested range 20-100
      int cooling = map(level, 0, 255, 20, 200);

      // Step 1.  Cool down every cell a little
      for ( int i = 0; i < num_leds; i++) {
        m_heat[i] = qsub8( m_heat[i],  random8(0, ((cooling * 10) / num_leds) + 2));
      }

      // Step 2.  Heat from each cell drifts 'up' and diffuses a little
      for ( int k = num_leds - 1; k >= 2; k--) {
        m_heat[k] = (m_heat[k - 1] + m_heat[k - 2] + m_heat[k - 2] ) / 3;
      }

      // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
      if ( random8() < m_sparking ) {
        int y = random8(3);
        m_heat[y] = qadd8( m_heat[y], random8(100, 150) );
      }

      // Step 4.  Map from heat cells to LED colors
      for ( int j = 0; j < num_leds; j++) {
        // Scale the heat value from 0-255 down to 0-240
        // for best results with color palettes.
        byte colorindex = scale8( m_heat[num_leds - j - 1], 170);
        Super::m_cell->setLED(j, colorindex);
      }
    }
};

template<uint8_t RANGE, uint8_t SPEED, typename Super>
class Disco : public Super
{
  private:
    uint8_t hue[LEDS_PER_CELL];
    uint8_t hue_level[LEDS_PER_CELL];
    uint8_t position[LEDS_PER_CELL];

  public:
    Disco(Cell * cell)
      : Super(cell)
    {}

    void init() {
      Super::init();
      for (int i = 0; i < LEDS_PER_CELL; i++) {
        // -- Random hues
        hue[i] = random8(RANGE);
        hue_level[i] = 0;
        // -- Random starting points on the curve
        position[i] = random8();
      }
    }

    void render(uint8_t level, uint32_t delta_t) {
      uint8_t adjusted_level = lerp8by8(0, 240, level);
      for (int i = 0; i < LEDS_PER_CELL; i++) {
        // -- Compute brightness a position on cosine wave
        uint8_t prev_pos = position[i];
        position[i] = prev_pos + SPEED; // Should give about 1/2 second per cycle
        uint8_t cur_pos = position[i];
        uint8_t wave = cos8(cur_pos);
        uint8_t brightness = lerp8by8(20, 255, wave);

        // -- Hue is incremented to match the level
        // uint8_t cur_hue_level = hue_level[i];
        // if (cur_hue_level < adjusted_level) hue_level[i] += min(2, adjusted_level - cur_hue_level);
        // if (cur_hue_level > adjusted_level) hue_level[i] = (cur_hue_level * 3 + adjusted_level) / 4;
        hue_level[i] = adjusted_level;

        Super::m_cell->setLED(i, hue[i] + hue_level[i], brightness);

        // -- At the end of each cycle, pick a new color
        if (prev_pos < 128 && cur_pos >= 128) {
          hue[i] = random8(RANGE);
        }
      }
    }
};

/*
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

    virtual void setLevel(int delta)
    {

    }

    virtual int getLevel()
    {
      return 0;
    }

    virtual const char * name()
    {
      return "Image";
    }
};
*/

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
        CRGB color_TR = getHueFromImage(x_whole + 1, y_whole, palette);
        uint8_t frac_TR = x_frac * (0x10 - y_frac);
        color_TR.nscale8_video(frac_TR);
        color += color_TR;
      }

      if (y_frac > 0) {
        // -- LED crosses into the adjacent pixel below
        CRGB color_BL = getHueFromImage(x_whole, y_whole + 1, palette);
        uint8_t frac_BL = (0x10 - x_frac) * y_frac;
        color_BL.nscale8_video(frac_BL);
        color += color_BL;
      }

      if (x_frac > 0 and y_frac > 0) {
        // -- LED crosses both boundaries into the corner LED
        CRGB color_BR = getHueFromImage(x_whole + 1, y_whole + 1, palette);
        uint8_t frac_BR = x_frac * y_frac;
        color_BR.nscale8_video(frac_BR);
        color += color_BR;
      }

      return color;
    }

    /** Cubic Hermite for discrete space

        Compute an interpolated value between B and C at a distance proportional
        to t. A and D are neighboring values that affect the curve near the edges
        (A is on the other side of B; D is on the other side of C):

          A ....... B ....... C ....... D
                    |<-t----->|
                       ^
                       | interpolated value

        This version is discretized to make it more efficient: it can only
        interpolate 9 points in between. In other words, t should be a value
        in the range [0,8].

        This choice makes certain computations fast. For example, instead of
        computing a * t * t * t (for values of t in [0,1]), we can compute
            a * (t/8) * (t/8) * (t/8)
          = a * t * t * t / (8 * 8 * 8)
          = (a * t * t * t) >> 9
    */

    uint8_t cubic8( uint8_t A, uint8_t B, uint8_t C, uint8_t D, uint16_t t)
    {
      uint32_t a = (D + 3 * B - 3 * C - A) / 2;
      uint32_t b = A + 2 * C - (5 * B + D) / 2;
      uint32_t c = (C - A) / 2;
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


template<typename Super>
class Image : public Super
{
  private:
    ImageView * m_image;

  public:
    Image(Cell * cell)
      : Super(cell),
        m_image(0)
    {}

    void init()
    {
      Super::init();
    }

    void render(uint8_t level, uint32_t delta_t)
    {
      // -- For each LED in this ring, sample the colors
      //    from the backing image to get a final color
      for (int i = 0; i < LEDS_PER_CELL; i++) {
        CRGB color = m_image->getImagePixel(i, Super::getX(), Super::getY(), Super::getPalette());
        Super::setRawLED(i, color);
      }
    }
};

// ===== Instantiations ===========================================

/** Pattern interface
 *  
 *  This class provides a pointer type that can be used to point to any instantiation
 *  of our pattern templates. Without this type, each instantiation has a different
 *  and unrelated type.
 */
class Pattern
{
  public:
    virtual void init() = 0;
    virtual uint8_t sense(uint32_t delta_t) = 0;
    virtual void render(uint8_t level, uint32_t delta_t) = 0;
};

template<typename Super>
class Wrap : public Super, public Pattern
{
  public:
    Wrap(Cell * cell)
      : Super(cell)
    {}

    void init() {
      Super::init();
    }

    uint8_t sense(uint32_t delta_t)  {
      return Super::sense(delta_t);
    }
    void render(uint8_t level, uint32_t delta_t) {
      Super::render(level, delta_t);
    }
};

typedef Wrap<
        Fade<150,
        ColorDot<500, 6000,
        SenseDecay<2, 12,
        IRSense< BasePattern > > > > > patGear;

typedef Wrap<
        ColorSolid< 
        BasePattern > > patSolidRaw;

typedef Wrap<
        ColorSolid< 
        SenseSineWave<2, 
        BasePattern > > > patSolidSine;

typedef Wrap<
        ColorSolid<
        SenseDecay<4, 12, 
        IRSense< 
        BasePattern > > > > patSolid;

typedef Wrap<Confetti<true, Fade<80, SenseDecay<4, 12, IRSense< BasePattern > > > > > patConfetti;

typedef Wrap< Fire< HeatColors< Fade<80, SenseDecay<2, 12, IRSense< BasePattern > > > > > > patFire;

// typedef Wrap< Image< IRSense< BasePattern > > > patImage;

// ===== Table ====================================================

#define NUM_PATTERNS 6
#define NUM_PALETTES 7

class ReactTable
{
  private:
    Cell *      m_Cell[NUM_CELLS];
    Pattern *   m_Pattern[NUM_CELLS];

    // ImageView   m_Image;

  public:

    ReactTable()
    {
      for (int i = 0; i < NUM_CELLS; i++) {
        m_Cell[i] = 0;
        m_Pattern[i] = 0;
      }

      /*
        m_patterns[0] = new SolidPattern();
        m_patterns[1] = new DotPattern(10,40);
        m_patterns[2] = new DiscoPattern(60,3);
        m_patterns[3] = new FirePattern(150);
        m_patterns[4] = new WavePattern();
        m_patterns[5] = new ImagePattern(&m_Image);
      */
    }

    void init()
    {
      int index = 0;
      //                        IRin IRch X   Y  LED
      m_Cell[index++] = new Cell( 0,  0,  0,  0, 60 );
      m_Cell[index++] = new Cell( 0,  1,  2,  0, 49 );
      m_Cell[index++] = new Cell( 0,  2,  4,  0, 38 );
      m_Cell[index++] = new Cell( 0,  3,  6,  0, 27 );
      m_Cell[index++] = new Cell( 0,  4,  8,  0, 16 );
      m_Cell[index++] = new Cell( 0,  5, 10,  0,  5 );
      m_Cell[index++] = new Cell( 0,  6,  1,  1, 54 );
      m_Cell[index++] = new Cell( 0,  7,  3,  1, 43 );
      m_Cell[index++] = new Cell( 0,  8,  5,  1, 32 );
      m_Cell[index++] = new Cell( 0,  9,  7,  1, 21 );
      m_Cell[index++] = new Cell( 0, 10,  9,  1, 10 );
      m_Cell[index++] = new Cell( 0, 11,  0,  2, 59 );
      m_Cell[index++] = new Cell( 0, 12,  2,  2, 48 );
      m_Cell[index++] = new Cell( 0, 13,  4,  2, 37 );
      m_Cell[index++] = new Cell( 0, 14,  6,  2, 26 );
      m_Cell[index++] = new Cell( 0, 15,  8,  2, 15 );
      m_Cell[index++] = new Cell( 1,  0, 10,  2,  4 );
      m_Cell[index++] = new Cell( 1,  1,  1,  3, 53 );
      m_Cell[index++] = new Cell( 1,  2,  3,  3, 42 );
      m_Cell[index++] = new Cell( 1,  3,  5,  3, 31 );
      m_Cell[index++] = new Cell( 1,  4,  7,  3, 20 );
      m_Cell[index++] = new Cell( 1,  5,  9,  3,  9 );
      m_Cell[index++] = new Cell( 1,  6,  0,  4, 58 );
      m_Cell[index++] = new Cell( 1,  7,  2,  4, 47 );
      m_Cell[index++] = new Cell( 1,  8,  4,  4, 36 );
      m_Cell[index++] = new Cell( 1,  9,  6,  4, 25 );
      m_Cell[index++] = new Cell( 1, 10,  8,  4, 14 );
      m_Cell[index++] = new Cell( 1, 11, 10,  4,  3 );
      m_Cell[index++] = new Cell( 1, 12,  1,  5, 52 );
      m_Cell[index++] = new Cell( 1, 13,  3,  5, 41 );
      m_Cell[index++] = new Cell( 1, 14,  5,  5, 30 );
      m_Cell[index++] = new Cell( 1, 15,  7,  5, 19 );
      m_Cell[index++] = new Cell( 2,  0,  9,  5,  8 );
      m_Cell[index++] = new Cell( 2,  1,  0,  6, 57 );
      m_Cell[index++] = new Cell( 2,  2,  2,  6, 46 );
      m_Cell[index++] = new Cell( 2,  3,  4,  6, 35 );
      m_Cell[index++] = new Cell( 2,  4,  6,  6, 24 );
      m_Cell[index++] = new Cell( 2,  5,  8,  6, 13 );
      m_Cell[index++] = new Cell( 2,  6, 10,  6,  2 );
      m_Cell[index++] = new Cell( 2,  7,  1,  7, 51 );
      m_Cell[index++] = new Cell( 2,  8,  3,  7, 40 );
      m_Cell[index++] = new Cell( 2,  9,  5,  7, 29 );
      m_Cell[index++] = new Cell( 2, 10,  7,  7, 18 );
      m_Cell[index++] = new Cell( 2, 11,  9,  7,  7 );
      m_Cell[index++] = new Cell( 2, 12,  0,  8, 56 );
      m_Cell[index++] = new Cell( 2, 13,  2,  8, 45 );
      m_Cell[index++] = new Cell( 2, 14,  4,  8, 34 );
      m_Cell[index++] = new Cell( 2, 15,  6,  8, 23 );
      m_Cell[index++] = new Cell( 3,  0,  8,  8, 12 );
      m_Cell[index++] = new Cell( 3,  1, 10,  8,  1 );
      m_Cell[index++] = new Cell( 3,  2,  1,  9, 50 );
      m_Cell[index++] = new Cell( 3,  3,  3,  9, 39 );
      m_Cell[index++] = new Cell( 3,  4,  5,  9, 28 );
      m_Cell[index++] = new Cell( 3,  5,  7,  9, 17 );
      m_Cell[index++] = new Cell( 3,  6,  9,  9,  6 );
      m_Cell[index++] = new Cell( 3,  7,  0, 10, 55 );
      m_Cell[index++] = new Cell( 3,  8,  2, 10, 44 );
      m_Cell[index++] = new Cell( 3,  9,  4, 10, 33 );
      m_Cell[index++] = new Cell( 3, 10,  6, 10, 22 );
      m_Cell[index++] = new Cell( 3, 11,  8, 10, 11 );
      m_Cell[index++] = new Cell( 3, 12, 10, 10,  0 );
      assert( index == NUM_CELLS);
    }

    /** Calibrate IR
        Read a series of values, using the average value as the max
    */
    void calibrate()
    {
      uint16_t total_ir[NUM_CELLS];

      for (int i = 0; i < NUM_CELLS; i++) total_ir[i] = 0;

      for (int rounds = 0; rounds < 4; rounds++) {
        for (int i = 0; i < NUM_CELLS; i++) {
          total_ir[i] += m_Cell[i]->rawIR();
        }
        delay(100);
      }

      for (int i = 0; i < NUM_CELLS; i++) {
        m_Cell[i]->setIRMax(total_ir[i] / 4);
        m_Cell[i]->setIRMin(140);
        Serial.println(total_ir[i] / 4);
      }
    }

    void setPattern()
    {
      for (int i = 0; i < NUM_CELLS; i++) {
        m_Pattern[i] = new patSolidSine(m_Cell[i]);
        m_Pattern[i]->init();
      }
    }

    void render(uint32_t delta_t)
    {
      for (int i = 0; i < NUM_CELLS; i++) {
        uint8_t level = m_Pattern[i]->sense(delta_t);
        m_Pattern[i]->render(level, delta_t);
      }
    }

};

// ===== Setup ====================================================

ReactTable g_Table;

void setup()
{
  // pinMode(LED_PIN, OUTPUT);
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);

  // pinMode(BUTTON_PIN, INPUT);
  // pinMode(ROTARY_A_PIN, INPUT_PULLUP);
  // pinMode(ROTARY_B_PIN, INPUT_PULLUP);

  for (int i = 0; i < 4; i++) {
    pinMode(IR_INPUT_PIN[i], INPUT);
  }

  pinMode(IR_CHANNEL_BIT_0, OUTPUT);
  pinMode(IR_CHANNEL_BIT_1, OUTPUT);
  pinMode(IR_CHANNEL_BIT_2, OUTPUT);
  pinMode(IR_CHANNEL_BIT_3, OUTPUT);

  Serial.begin(115200);
  delay(200);

  // pinMode(MIC_PIN, INPUT);

  // -- Create the FastLED show task
  xTaskCreatePinnedToCore(FastLEDshowTask, "FastLEDshowTask", 2048, NULL, 2, &FastLEDshowTaskHandle, FASTLED_SHOW_CORE);

  // -- Set up the LED rings (as a giant strip)
  //FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(g_LEDs, NUM_LEDS).setCorrection( TypicalLEDStrip );

  FastLED.addLeds<CHIPSET, LED_PIN_1, COLOR_ORDER>(g_LEDs, NUM_LEDS_1).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<CHIPSET, LED_PIN_2, COLOR_ORDER>(g_LEDs, NUM_LEDS_1, NUM_LEDS_2).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<CHIPSET, LED_PIN_3, COLOR_ORDER>(g_LEDs, NUM_LEDS_1 + NUM_LEDS_2, NUM_LEDS_3).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(g_Brightness);

  fill_solid(g_LEDs, NUM_LEDS, CRGB::Black);
  FastLEDshowESP32();
  delay(2000);

  fill_solid(g_LEDs, NUM_LEDS, CRGB::Yellow);
  FastLEDshowESP32();
  delay(2000);

  // -- Initialize the cells, calibrate, and set up the first pattern
  g_Table.init();
  g_Table.calibrate();
  g_Table.setPattern();

  fill_solid(g_LEDs, NUM_LEDS, CRGB::Green);
  FastLEDshowESP32(); // FastLED.show();
  delay(2000);

  fill_solid(g_LEDs, NUM_LEDS, CRGB::Black);
  FastLEDshowESP32(); // FastLED.show();
  delay(2000);

  uint32_t freemem = esp_get_free_heap_size();
  Serial.print("Free memory: ");
  Serial.println(freemem);
}

// ===== Main loop ================================================

uint8_t gHue = 0; // rotating "base color" used by many of the patterns
uint32_t cur_time = 0;
uint32_t last_time = 0;

void loop()
{
  cur_time = millis();
  g_Table.render(cur_time - last_time);
  FastLED.show();
  delay(50);
  last_time = cur_time;
}

