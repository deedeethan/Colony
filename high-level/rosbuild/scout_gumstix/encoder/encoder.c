#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/semaphore.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <asm/atomic.h>

MODULE_AUTHOR("Tom Mullins");
MODULE_LICENSE("GPL");

#define N_ENCODERS 4
const int enc_pins[N_ENCODERS][2] = {{78, 79}, {80, 81}, {82, 83}, {84, 85}};
struct class *enc_class = NULL;

struct encoder_device
{
  struct device *device;
  int pins[2];
  int irq[2];
  int status;
  atomic_t value;
  dev_t dev;
}
encs[N_ENCODERS];

static ssize_t enc_show(struct device *dev, struct device_attribute *attr,
    char *buf);
static ssize_t enc_store(struct device *dev, struct device_attribute *attr,
    const char *buf, size_t count);
static void enc_free(struct encoder_device *enc);
static int enc_init(dev_t dev, struct encoder_device *enc, const int *pins);

DEVICE_ATTR(ticks, S_IWUSR | S_IRUGO, enc_show, enc_store);

static ssize_t enc_show(struct device *dev, struct device_attribute *attr,
    char *buf)
{
  size_t nwritten;
  struct encoder_device *enc = dev_get_drvdata(dev);
  nwritten = scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&enc->value));
  return nwritten;
}

static ssize_t enc_store(struct device *dev, struct device_attribute *attr,
    const char *buf, size_t count)
{
  char *end;
  struct encoder_device *enc = dev_get_drvdata(dev);
  atomic_set(&enc->value, simple_strtol(buf, &end, 10));
  return count;
}

irqreturn_t intA(int irq, void *encp) {
  struct encoder_device *enc = encp;
  int this = gpio_get_value(enc->pins[0]);
  int other = gpio_get_value(enc->pins[1]);
  if (this == other) {
    atomic_inc(&enc->value);
  } else {
    atomic_dec(&enc->value);
  }
  return IRQ_HANDLED;
}

irqreturn_t intB(int irq, void *encp) {
  struct encoder_device *enc = encp;
  int this = gpio_get_value(enc->pins[1]);
  int other = gpio_get_value(enc->pins[0]);
  if (this == other) {
    atomic_dec(&enc->value);
  } else {
    atomic_inc(&enc->value);
  }
  return IRQ_HANDLED;
}

static int enc_init(dev_t dev, struct encoder_device *enc, const int *pins)
{
  int i, err;

  // initialize members
  enc->dev = dev;
  enc->status = 0;

  // register gpio and interrupts
  for (i = 0; i < 2; i++) {

    enc->pins[i] = pins[i];

    enc->irq[i] = gpio_to_irq(pins[i]);
    if (enc->irq[i] < 0) {
      printk("Error %d requesting irq number for gpio %d\n", enc->irq[i],
          enc->pins[i]);
      return 1;
    }

    err = gpio_direction_input(enc->pins[i]);
    if (err < 0) {
      printk("Error %d setting gpio %d to input\n", err, enc->pins[i]);
      return 1;
    }

    err = request_irq(enc->irq[i], i? intB:intA, 0, "encoder", enc);
    if (err < 0) {
      printk("Error %d requesting irq %d\n", err, enc->irq[i]);
      return 1;
    }

    err = irq_set_irq_type(enc->irq[i], IRQ_TYPE_EDGE_BOTH);
    if (err < 0) {
      printk("Error %d setting irq %d type\n", err, enc->irq[i]);
      return 1;
    }

    // TODO the error checking here does not properly clean up after itself
    // TODO perhaps we should use gpio_request? probably not necessary...
  }
  enc->status = 2;

  return 0;
}

int enc_create_dev(struct encoder_device *enc) {
  int err;

  // make file in /dev
  enc->device = device_create(enc_class, NULL, enc->dev, enc, "enc%d",
      MINOR(enc->dev));
  if (IS_ERR(enc->device))
  {
    err = PTR_ERR(enc->device);
    enc->device = NULL;
    enc_free(enc);
    return err;
  }
  enc->status = 3;

  device_create_file(enc->device, &dev_attr_ticks);
  // TODO error check
  enc->status = 4;

  return 0;
}

static void enc_free(struct encoder_device *enc)
{
  switch (enc->status)
  {
    case 4:
      device_remove_file(enc->device, &dev_attr_ticks);
    case 3:
      device_destroy(enc_class, enc->dev);
      enc->device = NULL;
    case 2:
      free_irq(enc->irq[0], enc);
      free_irq(enc->irq[1], enc);
  }
  enc->status = 0;
}

static void enc_exit_module(void)
{
  int i;
  for (i = 0; i < N_ENCODERS; i++)
  {
    enc_free(&encs[i]);
  }
  if (enc_class) {
    class_destroy(enc_class);
    enc_class = NULL;
  }
}
module_exit(enc_exit_module);

static int __init enc_init_module(void)
{
  int i, err;

  // initialize enc structures
  for (i = 0; i < N_ENCODERS; i++) {
    err = enc_init(MKDEV(0, i), &encs[i], enc_pins[i]);
    if (err)
    {
      printk(KERN_WARNING "Error %d initializing encoder %d\n", err, i);
      enc_exit_module();
      return err;
    }
  }

  // register our device class for files in /dev
  enc_class = class_create(THIS_MODULE, "encoder");
  if (IS_ERR(enc_class))
  {
    err = PTR_ERR(enc_class);
    enc_class = NULL;
    printk(KERN_WARNING "Error %d creating device class\n", err);
  }
  else
  {
    // create each device
    for (i = 0; i < N_ENCODERS; i++) {
      err = enc_create_dev(&encs[i]);
      if (err)
      {
        printk(KERN_WARNING "Error %d creating enc%d\n", err, i);
        enc_exit_module();
        return err;
      }
    }
  }

  return 0;
}
module_init(enc_init_module);
