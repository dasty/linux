/*
 * pcm3060.c  --  PCM3060 ALSA SoC Audio driver
 *
 * Copyright 2012 Michal Bachraty
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * TODO: This driver only support the PCM3060 in it's bootstraping mode.
 *       It will need so extra work to support software control via I2C
 *	 or SPI.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm3060.h>


#define PCM3060_REG_CONTROL		64
#	define PCM3060_MRST		(1<<7)
#	define PCM3060_SRST		(1<<6)
#	define PCM3060_ADPSV		(1<<5)
#	define PCM3060_DAPSV		(1<<4)
#	define PCM3060_SILICONVERSION	(1<<0)
/* all others reserved */

/* the attenuation registers take values from
 * -1 (0dB) to -127 (-63.0 dB) or others (muted) */
#define PCM3060_REG_DAC_ATTEN_LEFT		65
#define FIRSTREGISTER			PCM3060_REG_DAC_ATTEN_LEFT
#define PCM3060_REG_DAC_ATTEN_RIGHT	66

#define PCM3060_REG_DAC_CONTROL		68
#	define PCM3060_OVR1		(1<<6)
#	define PCM3060_MUTE_RIGHT	(1<<1)
#	define PCM3060_MUTE_LEFT	(1<<0)

#define	PCM3060_REG_DAC_OUTPHASE	68
#	define PCM3060_OUTPHASE_INVERTED	(1<<2)

#define PCM3060_REG_DAC_DEEMPH		69
#	define PCM3060_DIGDEEMPH_SHIFT	5
#	define PCM3060_DIGDEEMPH_MASK	(3<<PCM3060_DIGDEEMPH_SHIFT)
#	define PCM3060_DIGDEEMPH_CTRL	(1<<4)

#define PCM3060_REG_DAC_FILTER		69
#	define PCM3060_ROLLOFF_FAST	(1<<7)
//#	define PCM3060_DAC_FILTER_ALWAYS	(1<<2)

//#define	PCM3060_REG_DAC_OUTPHASE		71
//#	define PCM3060_OUTPHASE_INVERTED	(1<<0)

//// ADC attenuation
#define PCM3060_REG_ADC_ATTEN_LEFT 70
#define PCM3060_REG_ADC_ATTEN_RIGHT 71

#define PCM3060_REG_ADC_CONTROL		73

#define PCM3060_NR_REGS  (PCM3060_REG_ADC_CONTROL - PCM3060_REG_CONTROL + 1)
/*
 * Default PCM3060 power-up configuration
 * Array contains non-existing in hw register at address 0
 * Array do not include Chip ID, as codec driver does not use
 * registers read operations at all
 */

enum { bootstrap_mode = 0, i2c_mode } pcm3060_mode;

static const u8 pcm3060_dflt_reg[PCM3060_NR_REGS] = {
	PCM3060_MRST | PCM3060_SRST | PCM3060_ADPSV | PCM3060_DAPSV,
	255,
	255,
	0, /* reg. 67 */
	0,
	0,
	215,
	215,
	0,
	0,
};


struct pcm3060_private {
	/* SND_SOC_I2C or SND_SOC_SPI */
	enum snd_soc_control_type	bus_type;
	/* GPIO driving Reset pin, if any */
	int				gpio_nreset;
	/* power supply */
	struct regulator		*vd_supply;
	int mclk;
	uint8_t				mode;
};



static int pcm3060_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int format)
{
//	struct snd_soc_codec *codec = codec_dai->codec;
//	struct pcm3060_private *pcm3060 = snd_soc_codec_get_drvdata(codec);

	return 0;
}

static int pcm3060_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret;
	int val_a = 0;
	int val_b = 0;

	if (mute) {
		val_a = PCM3060_MUTE_RIGHT;
		val_b = PCM3060_MUTE_LEFT;
	}

	ret = snd_soc_update_bits(codec, PCM3060_REG_DAC_CONTROL,
			PCM3060_MUTE_RIGHT | PCM3060_MUTE_LEFT, val_a | val_b);
	if (ret < 0)
		return ret;

	return 0;
}

static int pcm3060_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct pcm3060_private *priv = snd_soc_codec_get_drvdata(codec);

	priv->mclk = freq;

	return 0;
}

static const struct snd_soc_dai_ops pcm3060_dai_ops = {
	.set_fmt	= pcm3060_set_dai_fmt,
	.set_sysclk	= pcm3060_set_dai_sysclk,
//	.hw_params	= pcm3060_hw_params,
	.digital_mute	= pcm3060_digital_mute,
};

static const DECLARE_TLV_DB_SCALE(vol_DAC_tlv, -12750, 50, 1);

static const struct snd_kcontrol_new pcm3060_snd_controls[] = {
	SOC_DOUBLE_R_TLV("Master Playback Volume",
			PCM3060_REG_DAC_ATTEN_LEFT,
			PCM3060_REG_DAC_ATTEN_RIGHT,
			0, 255 , 0, vol_DAC_tlv),
};

#define PCM3060_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
		SNDRV_PCM_RATE_96000)

#define PCM3060_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE)

struct snd_soc_dai_driver pcm3060_dai = {
	.name = "pcm3060-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = PCM3060_RATES,
		.formats = PCM3060_FORMATS,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = PCM3060_RATES,
		.formats = PCM3060_FORMATS,},
	.ops = &pcm3060_dai_ops,
	.symmetric_rates = 1,
};

#define pcm3060_soc_suspend	NULL
#define pcm3060_soc_resume	NULL


/* xxxxxxxxxxxxx*/

#ifdef CONFIG_OF
static const struct of_device_id pcm3060_dt_ids[] = {
	{ .compatible = "ti,pcm3060", },
	{ }
};
MODULE_DEVICE_TABLE(of, pcm3060_dt_ids);
#endif

static int pcm3060_probe(struct snd_soc_codec *codec)
{
	struct pcm3060_private *pcm3060 = snd_soc_codec_get_drvdata(codec);
	struct pcm3060_platform_data *pcm3060plat = codec->dev->platform_data;
	int ret;
	int gpio_nreset = -EINVAL;
	int mode;

	dev_info(codec->dev, "probing pcm3060\n");

	pcm3060->vd_supply = devm_regulator_get(codec->dev, "vd");
	if (IS_ERR(pcm3060->vd_supply)) {
		dev_warn(codec->dev, "missing vd supply\n");
		pcm3060->vd_supply = NULL;
	} else {
		ret = regulator_enable(pcm3060->vd_supply);
		if (ret < 0) {
			dev_err(codec->dev, "unable to enable regulator\n");
			return ret;
		}
	}

#ifdef CONFIG_OF
	if (of_match_device(pcm3060_dt_ids, codec->dev)) {
		gpio_nreset = of_get_named_gpio(codec->dev->of_node,
						"reset-gpio", 0);

		if (of_property_read_u32(codec->dev->of_node, "mode",
			     &mode) >= 0) {
			pcm3060->mode = mode;
		} else
			pcm3060->mode = i2c_mode;
	}
#endif

	if (pcm3060plat) {
		if (gpio_is_valid(pcm3060plat->gpio_nreset))
			gpio_nreset = pcm3060plat->gpio_nreset;
	}

	if (gpio_nreset >= 0)
		if (devm_gpio_request(codec->dev, gpio_nreset, "PCM3060 Reset"))
			gpio_nreset = -EINVAL;
	if (gpio_nreset >= 0) {
		/* Reset codec */
		gpio_direction_output(gpio_nreset, 0);
		udelay(1000);
		gpio_set_value(gpio_nreset, 1);
		/* Give the codec time to wake up */
		udelay(1000);
	}

	pcm3060->gpio_nreset = gpio_nreset;

	/*
	 * In case of I2C, chip address specified in board data.
	 * So cache IO operations use 8 bit codec register address.
	 */

	ret = snd_soc_codec_set_cache_io(codec, 8, 8,
			SND_SOC_I2C);

	if (ret) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
	dev_info(codec->dev, "going to add codec 3060\n");

	if (pcm3060->mode != bootstrap_mode) {
		/* bring codec to active state */
		ret = snd_soc_update_bits(codec, PCM3060_REG_CONTROL,
					  PCM3060_ADPSV | PCM3060_DAPSV, 0);
		if (ret < 0)
			return ret;
	} else
		return 0;

	return snd_soc_add_codec_controls(codec, pcm3060_snd_controls,
		ARRAY_SIZE(pcm3060_snd_controls));
}

static int pcm3060_remove(struct snd_soc_codec *codec)
{
	struct pcm3060_private *pcm3060 = snd_soc_codec_get_drvdata(codec);

	if (gpio_is_valid(pcm3060->gpio_nreset))
		/* Set codec to the reset state */
		gpio_set_value(pcm3060->gpio_nreset, 0);

	return 0;
};

static struct snd_soc_codec_driver soc_codec_dev_pcm3060 = {
	.probe			= pcm3060_probe,
	.remove			= pcm3060_remove,
	.suspend		= pcm3060_soc_suspend,
	.resume			= pcm3060_soc_resume,
	.reg_cache_default	= pcm3060_dflt_reg,
	.reg_cache_size		= ARRAY_SIZE(pcm3060_dflt_reg),
	.reg_word_size		= sizeof(pcm3060_dflt_reg[0]),
	.compress_type		= SND_SOC_FLAT_COMPRESSION,
};

/* xxxxxxxxxxxxx*/


#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
static const struct i2c_device_id pcm3060_i2c_id[] = {
	{"pcm3060", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, pcm3060_i2c_id);


static int pcm3060_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	int ret;
	struct pcm3060_private *pcm3060;

	pr_debug("probing pcm3060 i2c...\n");
	pcm3060 = devm_kzalloc(&client->dev, sizeof(*pcm3060), GFP_KERNEL);
	if (!pcm3060)
		return -ENOMEM;

	i2c_set_clientdata(client, pcm3060);
	pcm3060->bus_type = SND_SOC_I2C;

	ret = snd_soc_register_codec(&client->dev, &soc_codec_dev_pcm3060,
		&pcm3060_dai, 1);
	pr_info("registrered pcm3060 codec %d ...\n", ret);
	return ret;
}

static int pcm3060_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	return 0;
}


static struct i2c_driver pcm3060_i2c_driver = {
	.driver = {
		.name	= "pcm3060",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(pcm3060_dt_ids),
	},
	.id_table	= pcm3060_i2c_id,
	.probe		= pcm3060_i2c_probe,
	.remove		= pcm3060_i2c_remove,
};

#endif /*defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE) */


static int __init pcm3060_modinit(void)
{
	int ret;

	pr_debug("pcm3060_modinit\n");
#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	ret = i2c_add_driver(&pcm3060_i2c_driver);
	if (ret) {
		pr_err("Failed to register pcm3060 I2C driver: %d\n", ret);
		return ret;
	}
#endif
	return 0;
}

module_init(pcm3060_modinit);

static void __exit pcm3060_modexit(void)
{

#if defined(CONFIG_I2C) || defined(CONFIG_I2C_MODULE)
	i2c_del_driver(&pcm3060_i2c_driver);
#endif
}

module_exit(pcm3060_modexit);

MODULE_DESCRIPTION("ASoC PCM3060 driver");
MODULE_AUTHOR("Michal Bachraty");
MODULE_LICENSE("GPL");
