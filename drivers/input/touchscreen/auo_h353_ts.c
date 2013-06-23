#if defined (CONFIG_ACER_DEBUG)
#define DEBUG
#endif

#include <linux/input.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <linux/input/auo_h353_ts.h>
#include <mach/board.h>

#define TS_DRIVER_NAME "auo-touch"

#define AUO_TS_X_MIN             0
#define AUO_TS_X_MAX             480
#define AUO_TS_Y_MIN             0
#define AUO_TS_Y_MAX             800

#define SLEEP_MODE_REG           0x70

#define INTERRUPT_MODE_REG       0x6e
#define PERIODICAL_MODE          0x0c
#define COORDINATE_COMPARE_MODE  0x0d

#define SENSITIVITY_REG          0x67
#define SENSITIVITY              75

#define NOISE_REG                0x37
#define NOISE                    75

#define VERSION_REG              0x7c
#define SUB_VERSION_REG          0x78

#define SAMPLING_NUM_REG         0x3a

#define RESET_REG                0x2e
#define USE_FS                   1

#define gpio_output_enable(gpio, en) do { \
    if (en == 0) { \
	gpio_direction_input(gpio); \
    } \
    else { \
	gpio_direction_output(gpio, 0); \
    } } while (0)

enum {
	VERSION_4_3 = 1027,
};

typedef enum
{
	ACTIVE,
	SUSPEND,
	SUSPENDING,
	RESUME,
	INIT,
} ts_status;

struct h353vl01_data{
	struct work_struct work;
	struct i2c_client *client;
	struct input_dev *input;
	ts_status status;
	int version;
	int gpio_ts_irq;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

static struct h353vl01_data *h353_data;

#if USE_FS

static uint8_t ts_atoi(const char *name)
{
    uint8_t val = 0;

    for (;; name++) {
	switch (*name) {
	    case '0' ... '9':
		val = 10*val+(*name-'0');
		break;
	    default:
		return val;
	}
    }
}

static ssize_t set_ts_sensitivity(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	uint8_t sensitivity[3] = {SENSITIVITY_REG,ts_atoi(buf),ts_atoi(buf)};
	pr_info("[TS] Sensitivity : X = %d  Y = %d\n",sensitivity[1],sensitivity[2]);
	if (3 != i2c_master_send(h353_data->client, sensitivity, 3))
		pr_err("[TS] Set sensitivity error\n");
	return count;
}

static ssize_t set_ts_noise(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	uint8_t noise_value[3] = {NOISE_REG,ts_atoi(buf),ts_atoi(buf)};
	pr_info("[TS] Sensitivity : Noise_X = %d  Noise_Y = %d\n",
				noise_value[1],noise_value[2]);
	if (3 != i2c_master_send(h353_data->client, noise_value, 3))
		pr_err("[TS] Set Noise error\n");
	return count;
}

static struct device_attribute ts_attrs =
__ATTR(sensitivity, S_IRWXUGO,NULL, set_ts_sensitivity);

static struct device_attribute ts_noise_attrs =
__ATTR(noise, S_IRWXUGO,NULL, set_ts_noise);

#endif

static int set_mode(ts_status status)
{
	uint8_t interrupt_mode[2] = {INTERRUPT_MODE_REG,0};
	uint8_t sleep_mode[2] = {SLEEP_MODE_REG,0};
	uint8_t sensitivity_value[3] = {SENSITIVITY_REG,SENSITIVITY,SENSITIVITY};
	uint8_t noise_value[3] = {NOISE_REG,NOISE,NOISE};
	uint8_t sampling_num[2] = {SAMPLING_NUM_REG,2};
	uint8_t tp_version[2] = {VERSION_REG,SUB_VERSION_REG};
	uint8_t reset[2] = {RESET_REG,1};
	uint8_t buf = 0;

	switch(status){
	case SUSPENDING:
		/* Set Interrupt Mode to Sensing Periodical Mode*/
		if(h353_data->version < VERSION_4_3) {
			interrupt_mode[1] = PERIODICAL_MODE;
			if (2 != i2c_master_send(h353_data->client, interrupt_mode, 2))
				goto i2c_err;
		}
		/* Change from Active Mode to Sleep Mode */
		if (1 != i2c_master_send(h353_data->client, &sleep_mode[0], 1))
			goto i2c_err;
		if (1 !=i2c_master_recv(h353_data->client, &buf, 1))
			goto i2c_err;
		sleep_mode[1] = ( buf & 0xfc ) | 0x2;
		if (2 != i2c_master_send(h353_data->client, sleep_mode, 2))
			goto i2c_err;
		h353_data->status = SUSPEND;
		break;
	case RESUME:
		if(h353_data->version >= VERSION_4_3) {
			gpio_output_enable(h353_data->gpio_ts_irq, 1);
			gpio_set_value(h353_data->gpio_ts_irq, 0);
			gpio_set_value(h353_data->gpio_ts_irq, 1);
			msleep(40);
			gpio_set_value(h353_data->gpio_ts_irq, 0);
			gpio_output_enable(h353_data->gpio_ts_irq, 0);
			msleep(20);
		}
		/* Reset TouchScreen */
		if (2 != i2c_master_send(h353_data->client, reset, 2))
			goto i2c_err;
		msleep(20);

		/* Change from Sleep Mode to Active Mode */
		if (1 != i2c_master_send(h353_data->client, &sleep_mode[0], 1))
			goto i2c_err;
		if (1 !=i2c_master_recv(h353_data->client, &buf, 1))
			goto i2c_err;
		if (h353_data->version < VERSION_4_3){
			sleep_mode[1] = buf & 0xfc;
			if (2 != i2c_master_send(h353_data->client, sleep_mode, 2))
				goto i2c_err;

			/* Set Interrupt Mode to Coordinate Compare Mode*/
			interrupt_mode[1] = COORDINATE_COMPARE_MODE;
			if (2 != i2c_master_send(h353_data->client, interrupt_mode, 2))
				goto i2c_err;
		}
		h353_data->status = ACTIVE;
		break;
	case INIT:
		interrupt_mode[1] = COORDINATE_COMPARE_MODE;
		/* Noise & sensitivity setting */
		if (3 != i2c_master_send(h353_data->client, sensitivity_value, 3))
			pr_err("[TS] Set sensitivity error\n");
		if (3 != i2c_master_send(h353_data->client, noise_value, 3))
			pr_err("[TS] Set Noise error\n");
		if (2 != i2c_master_send(h353_data->client, sampling_num, 2))
                        pr_err("[TS] Set Sampling Number error\n");

		/* Read version */
		if (1 != i2c_master_send(h353_data->client, &tp_version[0], 1))
			goto i2c_err;
		if (1 !=i2c_master_recv(h353_data->client, &buf, 1))
			goto i2c_err;
		h353_data->version |= (int)((buf&0xf)<<8);
		if (1 != i2c_master_send(h353_data->client, &tp_version[1], 1))
			goto i2c_err;
		if (1 !=i2c_master_recv(h353_data->client, &buf, 1))
			goto i2c_err;
		h353_data->version |= buf;
		pr_info("[TS] version = %x\n",h353_data->version);

		if (2 != i2c_master_send(h353_data->client, interrupt_mode, 2))
			goto i2c_err;
		h353_data->status = ACTIVE;
		break;
	default:
		break;

	}
	return 0;
i2c_err:
	pr_err("[TS] %s error (%d)\n",__func__,status);
	return -ENXIO;
}

static void h353vl01_work_func(struct work_struct *work)
{
	unsigned int coord[2][2]= {{0}};
	uint8_t data_addr = 0x40;
	uint8_t buf[8] = { 0 };
	int pressed = 0;
	int finger2_pressed = 0;
	int width = 0;

	if (h353_data->status != ACTIVE) {
		set_mode(h353_data->status);
		return;
	};

	if (1 != i2c_master_send(h353_data->client, &data_addr, 1))
		goto i2c_err;
	if (8 !=i2c_master_recv(h353_data->client, buf, 8))
		goto i2c_err;

	/* coord[0]: finger1, coord[1]: finger2 */
	coord[0][0] = buf[0] + (buf[4]*256);
	coord[0][1] = buf[1] + (buf[5]*256);
	coord[1][0] = buf[2] + (buf[6]*256);
	coord[1][1] = buf[3] + (buf[7]*256);

	pressed = (coord[0][0]||coord[0][1]) ? 1 : 0;
	finger2_pressed = (coord[1][0]||coord[1][1]) ? 1: 0;

	// New MT framework
	if (pressed) {
		input_report_abs(h353_data->input, ABS_MT_POSITION_X, coord[0][0]);
		input_report_abs(h353_data->input, ABS_MT_POSITION_Y, coord[0][1]);

		/* Calculate event width */
		if (finger2_pressed) {
			/* Report the width according to the abs distance of x-axis */
			width = abs((coord[0][0] - coord[1][0]));
		}
	}
	input_report_abs(h353_data->input, ABS_MT_WIDTH_MAJOR, width);
	input_report_abs(h353_data->input, ABS_MT_TOUCH_MAJOR, pressed);
	input_mt_sync(h353_data->input);

	// Report second pointer
	if (finger2_pressed) {
		input_report_abs(h353_data->input, ABS_MT_POSITION_X, coord[1][0]);
		input_report_abs(h353_data->input, ABS_MT_POSITION_Y, coord[1][1]);
	}
	input_report_abs(h353_data->input, ABS_MT_WIDTH_MAJOR, width);
	input_report_abs(h353_data->input, ABS_MT_TOUCH_MAJOR, finger2_pressed);
	input_mt_sync(h353_data->input);

	input_sync(h353_data->input);

	return;
i2c_err:
	pr_err("[TS] Work i2c error\n");
}

static irqreturn_t h353vl01_ts_interrupt(int irq, void *dev_id)
{
	if(h353_data->status == SUSPEND)
		return IRQ_HANDLED;

	schedule_work(&h353_data->work);

	return IRQ_HANDLED;
}

static int __init h353vl01_register_input(struct input_dev *input)
{
	input->name = TS_DRIVER_NAME;
	input->id.bustype = BUS_I2C;
	input->evbit[0] =
		BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 1, 0, 0);
	input_set_abs_params(input, ABS_MT_WIDTH_MAJOR, 0, AUO_TS_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, AUO_TS_X_MIN, AUO_TS_X_MAX, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, AUO_TS_Y_MIN, AUO_TS_Y_MAX, 0, 0);
	return input_register_device(input);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void h353vl01_early_suspend(struct early_suspend *h)
{
	disable_irq(h353_data->client->irq);
	h353_data->status = SUSPENDING;
	set_mode(h353_data->status);
	if (h353_data->status == SUSPENDING)
		pr_err("[TS] %s error\n",__func__);
	enable_irq(h353_data->client->irq);
}

void h353vl01_early_resume(struct early_suspend *h)
{
	//pr_debug("[TS] Enter %s and resume Done\n",__func__);
	pr_info("[TS] %s++\n", __func__);
	h353_data->status = RESUME;
	if(h353_data->version >= VERSION_4_3) {
		int nCount = 0;
		disable_irq(h353_data->client->irq);
		do{
			set_mode(h353_data->status);
			nCount++;
			if(nCount == 10)
				pr_err("[TS] resume error");
		}while(h353_data->status == RESUME && nCount <10);
		enable_irq(h353_data->client->irq);
	}
	pr_info("[TS] %s--\n", __func__);
}
#endif

static int h353vl01_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret;
	struct auo_h353_ts_platform_data *pdata = client->dev.platform_data;

	h353_data = kzalloc(sizeof(struct h353vl01_data),GFP_KERNEL);
	if (h353_data == NULL)
		return -ENOMEM;

	h353_data->client = client;
	if (!pdata) {
		dev_err(&client->dev, "%s: platform data is required\n",
								__func__);
		return -EINVAL;
	}

	h353_data->gpio_ts_irq = pdata->gpio_ts_irq;

	ret = gpio_request(h353_data->gpio_ts_irq, "TS_IRQ");
	if (ret) {
		dev_err(&client->dev, "%s: gpio_request for TS_IRQ failed\n",
								__func__);
		return ret;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENOTSUPP;

	strlcpy(client->name, TS_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, h353_data);

	INIT_WORK(&h353_data->work, h353vl01_work_func);

	h353_data->input = input_allocate_device();
	if (h353_data->input == NULL)
		return -ENOMEM;

	if (h353vl01_register_input(h353_data->input))
		goto set_mode_err;

	if (0 != set_mode(INIT))
		goto set_mode_err;

	if (client->irq) {
		if (request_irq(client->irq, h353vl01_ts_interrupt,
				IRQF_TRIGGER_RISING | IRQF_DISABLED,
				TS_DRIVER_NAME, h353_data))
		goto request_irq_err;
	}

#if USE_FS
	if(device_create_file(&client->dev, &ts_attrs))
		pr_err("[TS] device_create_file ts_attrs error \n");

	if(device_create_file(&client->dev, &ts_noise_attrs))
		pr_err("[TS] device_create_file ts_noise_attrs error \n");
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	h353_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	h353_data->early_suspend.suspend = h353vl01_early_suspend;
	h353_data->early_suspend.resume = h353vl01_early_resume;
	register_early_suspend(&h353_data->early_suspend);
#endif
	return 0;
request_irq_err:
	free_irq(client->irq, h353_data);
set_mode_err:
	input_free_device(h353_data->input);
	kfree(h353_data);
	return -ENOTSUPP;
}

static int h353vl01_remove(struct i2c_client *client)
{
	struct h353vl01_data *tp = i2c_get_clientdata(client);
	input_unregister_device(tp->input);
	free_irq(client->irq, tp);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&tp->early_suspend);
#endif
	kfree(h353_data);
	return 0;
}

static const struct i2c_device_id h353vl01_id[] = {
	{ TS_DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver h353vl01_driver = {
	.probe		= h353vl01_probe,
	.remove		= h353vl01_remove,
	.id_table	= h353vl01_id,
	.driver		= {
		.name = TS_DRIVER_NAME,
	},
};

static int __init h353vl01_init(void)
{
	pr_debug("[TS] Enter %s \n",__func__);
	return i2c_add_driver(&h353vl01_driver);
}

static void __exit h353vl01_exit(void)
{
	i2c_del_driver(&h353vl01_driver);
}

module_init(h353vl01_init);
module_exit(h353vl01_exit);

MODULE_AUTHOR("Allan Lin <Allan_Lin@acer.com.tw>");
MODULE_DESCRIPTION("AUO h353vl01 driver");
MODULE_LICENSE("GPL v2");