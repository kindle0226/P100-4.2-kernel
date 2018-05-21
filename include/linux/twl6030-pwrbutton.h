

struct twl6030_pwr_button {
	struct input_dev *input_dev;
	struct device *dev;
	int report_key;
};
extern struct twl6030_pwr_button pbutton;
