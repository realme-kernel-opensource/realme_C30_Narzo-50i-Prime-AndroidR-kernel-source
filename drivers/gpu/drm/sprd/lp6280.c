/*
 * Regulator driver for LP6280 Bias chip
 *
 */

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#define LP6280_REG_VPOS			0x00
#define LP6280_REG_VNEG			0x01
#define LP6280_REG_APPS_DISP_DISN	0x03

#define LP6280_VOUT_MASK		0x1F
#define LP6280_VOUT_N_VOLTAGE		0x15
#define LP6280_VOUT_VMIN		4000
#define LP6280_VOUT_VMAX		6000
#define LP6280_VOUT_STEP		100

struct lp6280_data {
	struct device *dev;
	struct i2c_client *i2c;
};

static struct lp6280_data *lp6280;

/* set voltage in mV */
void lp6280_set_voltage(unsigned int vol)
{
	int ret;
	u8 val;

	if (vol > LP6280_VOUT_VMAX) {
		dev_err(lp6280->dev, "voltage out of range\n");
		return;
	}
	val = ((vol - LP6280_VOUT_VMIN) / LP6280_VOUT_STEP) & LP6280_VOUT_MASK;

	ret = i2c_smbus_write_byte_data(lp6280->i2c, LP6280_REG_VPOS, val);
	if (ret < 0)
		dev_err(lp6280->dev, "write reg [0x%02x, %u] failed\n", LP6280_REG_VPOS, val);

	ret = i2c_smbus_write_byte_data(lp6280->i2c, LP6280_REG_VNEG, val);
	if (ret < 0)
		dev_err(lp6280->dev, "write reg [0x%02x, %u] failed\n", LP6280_REG_VNEG, val);

}
EXPORT_SYMBOL_GPL(lp6280_set_voltage);

void lp6280_set_current_for_tablet(void)
{
	int ret = -1;
	int val = i2c_smbus_read_byte_data(lp6280->i2c,LP6280_REG_APPS_DISP_DISN);
	if (val >= 0)
		ret = i2c_smbus_write_byte_data(lp6280->i2c, LP6280_REG_APPS_DISP_DISN, val | 0x40);

	if (val < 0 || ret < 0)
		dev_err(lp6280->dev, "set current failed! [0x%02x, %d, %d]\n", LP6280_REG_APPS_DISP_DISN, val, ret);
}
EXPORT_SYMBOL_GPL(lp6280_set_current_for_tablet);

static int lp6280_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	lp6280 = devm_kzalloc(&i2c->dev, sizeof(*lp6280), GFP_KERNEL);
	if (!lp6280)
		return -ENOMEM;

	dev_err(&i2c->dev, "%s: start\n", __func__);

	lp6280->i2c = i2c;
	lp6280->dev = &i2c->dev;

	i2c_set_clientdata(i2c, lp6280);

	return 0;
}

static const struct i2c_device_id lp6280_i2c_id[] = {
	{ "lcd_bias_lp6280", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lp6280_i2c_id);

static struct i2c_driver lp6280_i2c_driver = {
	.driver = {
		.name = "lp6280",
	},
	.probe    = lp6280_i2c_probe,
	.id_table = lp6280_i2c_id,
};

module_i2c_driver(lp6280_i2c_driver);
MODULE_LICENSE("GPL");
