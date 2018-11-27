# F746GDISCO_DCMI
AN5020 DCMI Example: STM32FGDISCOVERY with STM32F4DIS-CAM (OV9655)

- IDE: Atollic TrueSTUDIO for STM32 with STM32CubeMX
- Display: ROCKTECH, 4.3-inch 480x272 LCD-TFT
- Camera: STM32F4DIS-CAM (OV9655, 1280x1024 SXGA)

Camera Module:
1. Picture Size
2. Frame Rate
3. Data Format
4. Scaling Image Output:
The OV9656 is capable of scaling down the image size from VGA to 40x30. By using register bits 
    * COM14[1] (0x3E)
    * COM16[0] (0x41)
    * POIDX (0x72)
    * XINDX (0x74)
    * YINDX (0x75)
At certain image sizes, HREF is not
consistent in a frame.

Notes:
1. use HSE 25MHz(X2, RCC_HSE_BYPASS) as the clock source
2. enable (LTDC,FMC) in STM32CubeMX
3. put font24.c in src folder, and no need to include the file in main.c
4. the LCD-TFT might burn if using the 320x240 resolution
5. some issues with STM32CubeMX code regeneration

2018.11.28
1. generate codes with STM32CubeMX and add device files 
