// SPDX-License-Identifier: GPL-2.0-only
/*
 *  GPIO driven matrix keyboard driver
 *
 *  Copyright (c) 2008 Marek Vasut <marek.vasut@gmail.com>
 *
 *  Based on corgikbd.c
 * 
 *  Modified 2020 by Michael Linsenmeier (Doctor Volt) <michalin70@gmail.com>
 *  - Based on matrix_keypad.c 
 *  - Support of modifier keys like Shift, CTRL, ALT...
 */

#include "c2020.h"
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/types.h>

struct c2020 {
  const struct c2020_platform_data *pdata;
  struct input_dev *input_dev;
  unsigned int row_shift;

  DECLARE_BITMAP(disabled_gpios, MATRIX_MAX_ROWS);

  uint32_t last_key_state[MATRIX_MAX_COLS];
  struct delayed_work work;
  spinlock_t lock;
  bool scan_pending;
  bool stopped;
  bool gpio_all_disabled;
};

/*bool c2020_is_modifier_key(const struct c2020_platform_data *pdata, int code)
{
  int i;
  for(i = 0; i < pdata->num_col_gpios * pdata->num_row_gpios; i++)
  {
    //printk("keycode: %x, code: %x", pdata->keycodes[i], code);
    if(((pdata->keycodes[i] & 0xfff) == code) && (pdata->keycodes[i] & 0x1000))
      return true;
  }
  return false;
}*/

bool c2020_map_keys(struct input_dev *input_dev,
                    const struct c2020_platform_data *pdata) {
  unsigned short *keymap = input_dev->keycode;
  unsigned int i, row, col; 
  unsigned short code;
  unsigned int row_shift = get_count_order(pdata->num_col_gpios);
  for (i = 0; i < pdata->num_col_gpios * pdata->num_row_gpios; i++) {
    unsigned int key = pdata->keycodes[i];
    row = KEY_ROW(key);
    col = KEY_COL(key);
    code = KEY_VAL(key);
    //printk("i: %d, row: %d, col: %d, code: %d", i, row, col, code);
    if (row >= pdata->num_row_gpios || col >= pdata->num_col_gpios) {
      dev_err(
          input_dev->dev.parent,
          "%s: invalid keymap entry 0x%x (row: %d, col: %d, rows: %d, cols: "
          "%d)\n",
          __func__, key, row, col, pdata->num_row_gpios, pdata->num_col_gpios);
      return false;
    }

    keymap[MATRIX_SCAN_CODE(row, col, row_shift)] = code;
    __set_bit(code, input_dev->keybit);
  }
  return true;
}

/*
 * NOTE: If drive_inactive_cols is false, then the GPIO has to be put into
 * HiZ when de-activated to cause minmal side effect when scanning other
 * columns. In that case it is configured here to be input, otherwise it is
 * driven with the inactive value.
 */
static void __activate_col(const struct c2020_platform_data *pdata, int col,
                           bool on) {
  bool level_on = !pdata->active_low;

  if (on) {
    gpio_direction_output(pdata->col_gpios[col], level_on);
  } else {
    gpio_set_value_cansleep(pdata->col_gpios[col], !level_on);
    if (!pdata->drive_inactive_cols)
      gpio_direction_input(pdata->col_gpios[col]);
  }
}

static void activate_col(const struct c2020_platform_data *pdata, int col,
                         bool on) {
  __activate_col(pdata, col, on);

  if (on && pdata->col_scan_delay_us)
    udelay(pdata->col_scan_delay_us);
}

static void activate_all_cols(const struct c2020_platform_data *pdata,
                              bool on) {
  int col;

  for (col = 0; col < pdata->num_col_gpios; col++)
    __activate_col(pdata, col, on);
}

static bool row_asserted(const struct c2020_platform_data *pdata, int row) {
  return gpio_get_value_cansleep(pdata->row_gpios[row]) ? !pdata->active_low
                                                        : pdata->active_low;
}

static void enable_row_irqs(struct c2020 *keypad) {
  const struct c2020_platform_data *pdata = keypad->pdata;
  int i;

  if (pdata->clustered_irq > 0)
    enable_irq(pdata->clustered_irq);
  else {
    for (i = 0; i < pdata->num_row_gpios; i++)
      enable_irq(gpio_to_irq(pdata->row_gpios[i]));
  }
}

static void disable_row_irqs(struct c2020 *keypad) {
  const struct c2020_platform_data *pdata = keypad->pdata;
  int i;

  if (pdata->clustered_irq > 0)
    disable_irq_nosync(pdata->clustered_irq);
  else {
    for (i = 0; i < pdata->num_row_gpios; i++)
      disable_irq_nosync(gpio_to_irq(pdata->row_gpios[i]));
  }
}

/*
 * This gets the keys from keyboard and reports it to input subsystem
 */
static void c2020_scan(struct work_struct *work) {
  struct c2020 *keypad = container_of(work, struct c2020, work.work);
  struct input_dev *input_dev = keypad->input_dev;
  const unsigned short *keycodes = input_dev->keycode;
  const struct c2020_platform_data *pdata = keypad->pdata;
  uint32_t new_state[MATRIX_MAX_COLS];
  int row, col;
  static int code;
  static bool pressed = false;

  //printk("c2020_probe");

  /* de-activate all columns for scanning */
  activate_all_cols(pdata, false);

  memset(new_state, 0, sizeof(new_state));

  /* assert each column and read the row status out */
  for (col = 0; col < pdata->num_col_gpios; col++) {

    activate_col(pdata, col, true);

    for (row = 0; row < pdata->num_row_gpios; row++)
      new_state[col] |= row_asserted(pdata, row) ? (1 << row) : 0;

    activate_col(pdata, col, false);
    // printk("%d: %x ", col, new_state[col]);
  }

  for (col = 0; col < pdata->num_col_gpios; col++) {
    uint32_t bits_changed;

    bits_changed = keypad->last_key_state[col] ^ new_state[col];
    if (bits_changed == 0)
      continue;

    for (row = 0; row < pdata->num_row_gpios; row++) {
      if ((bits_changed & (1 << row)) == 0)
        continue;

      code = MATRIX_SCAN_CODE(row, col, keypad->row_shift);
      input_event(input_dev, EV_MSC, MSC_SCAN, code);
      input_report_key(input_dev, keycodes[code], new_state[col] & (1 << row));

      //if (c2020_is_modifier_key(pdata, keycodes[code]))
      pressed = new_state[col];
    }
  }
  input_sync(input_dev);

  memcpy(keypad->last_key_state, new_state, sizeof(new_state));

  activate_all_cols(pdata, true);

  if (pressed) {
    //call c2020_scan again
    schedule_delayed_work(&keypad->work,
                          msecs_to_jiffies(pdata->modifier_scancycle_ms));
    return;
  }
  
  /* Enable IRQs again */
  spin_lock_irq(&keypad->lock);
  keypad->scan_pending = false;
  enable_row_irqs(keypad);
  spin_unlock_irq(&keypad->lock);
}

static irqreturn_t c2020_interrupt(int irq, void *id) {
  struct c2020 *keypad = id;
  unsigned long flags;

  spin_lock_irqsave(&keypad->lock, flags);

  /*
   * See if another IRQ beaten us to it and scheduled the
   * scan already. In that case we should not try to
   * disable IRQs again.
   */
  if (unlikely(keypad->scan_pending || keypad->stopped))
    goto out;

  disable_row_irqs(keypad);
  keypad->scan_pending = true;
  schedule_delayed_work(&keypad->work,
                        msecs_to_jiffies(keypad->pdata->debounce_ms));

out:
  spin_unlock_irqrestore(&keypad->lock, flags);
  return IRQ_HANDLED;
}

static int c2020_start(struct input_dev *dev) {
  struct c2020 *keypad = input_get_drvdata(dev);
  keypad->stopped = false;
  mb();

  /*
   * Schedule an immediate key scan to capture current key state;
   * columns will be activated and IRQs be enabled after the scan.
   */
  schedule_delayed_work(&keypad->work, 0);

  return 0;
}

static void c2020_stop(struct input_dev *dev) {
  struct c2020 *keypad = input_get_drvdata(dev);

  spin_lock_irq(&keypad->lock);
  keypad->stopped = true;
  spin_unlock_irq(&keypad->lock);

  flush_delayed_work(&keypad->work);
  /*
   * c2020_scan() will leave IRQs enabled;
   * we should disable them now.
   */
  disable_row_irqs(keypad);
}

#ifdef CONFIG_PM_SLEEP
static void c2020_enable_wakeup(struct c2020 *keypad) {
  const struct c2020_platform_data *pdata = keypad->pdata;
  unsigned int gpio;
  int i;

  if (pdata->clustered_irq > 0) {
    if (enable_irq_wake(pdata->clustered_irq) == 0)
      keypad->gpio_all_disabled = true;
  } else {

    for (i = 0; i < pdata->num_row_gpios; i++) {
      if (!test_bit(i, keypad->disabled_gpios)) {
        gpio = pdata->row_gpios[i];

        if (enable_irq_wake(gpio_to_irq(gpio)) == 0)
          __set_bit(i, keypad->disabled_gpios);
      }
    }
  }
}

static void c2020_disable_wakeup(struct c2020 *keypad) {
  const struct c2020_platform_data *pdata = keypad->pdata;
  unsigned int gpio;
  int i;

  if (pdata->clustered_irq > 0) {
    if (keypad->gpio_all_disabled) {
      disable_irq_wake(pdata->clustered_irq);
      keypad->gpio_all_disabled = false;
    }
  } else {
    for (i = 0; i < pdata->num_row_gpios; i++) {
      if (test_and_clear_bit(i, keypad->disabled_gpios)) {
        gpio = pdata->row_gpios[i];
        disable_irq_wake(gpio_to_irq(gpio));
      }
    }
  }
}

static int c2020_suspend(struct device *dev) {
  struct platform_device *pdev = to_platform_device(dev);
  struct c2020 *keypad = platform_get_drvdata(pdev);

  c2020_stop(keypad->input_dev);

  if (device_may_wakeup(&pdev->dev))
    c2020_enable_wakeup(keypad);

  return 0;
}

static int c2020_resume(struct device *dev) {
  struct platform_device *pdev = to_platform_device(dev);
  struct c2020 *keypad = platform_get_drvdata(pdev);

  if (device_may_wakeup(&pdev->dev))
    c2020_disable_wakeup(keypad);

  c2020_start(keypad->input_dev);

  return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(c2020_pm_ops, c2020_suspend, c2020_resume);

static int c2020_init_gpio(struct platform_device *pdev, struct c2020 *keypad) {
  const struct c2020_platform_data *pdata = keypad->pdata;
  int i, err;

  /* initialized strobe lines as outputs, activated */
  for (i = 0; i < pdata->num_col_gpios; i++) {
    err = gpio_request(pdata->col_gpios[i], "matrix_kbd_col");
    if (err) {
      dev_err(&pdev->dev, "failed to request GPIO%d for COL%d\n",
              pdata->col_gpios[i], i);
      goto err_free_cols;
    }

    gpio_direction_output(pdata->col_gpios[i], !pdata->active_low);
  }

  for (i = 0; i < pdata->num_row_gpios; i++) {
    err = gpio_request(pdata->row_gpios[i], "matrix_kbd_row");
    if (err) {
      dev_err(&pdev->dev, "failed to request GPIO%d for ROW%d\n",
              pdata->row_gpios[i], i);
      goto err_free_rows;
    }

    gpio_direction_input(pdata->row_gpios[i]);
  }

  if (pdata->clustered_irq > 0) {
    err = request_any_context_irq(pdata->clustered_irq, c2020_interrupt,
                                  pdata->clustered_irq_flags, "matrix-keypad",
                                  keypad);
    if (err < 0) {
      dev_err(&pdev->dev, "Unable to acquire clustered interrupt\n");
      goto err_free_rows;
    }
  } else {
    for (i = 0; i < pdata->num_row_gpios; i++) {
      err = request_any_context_irq(
          gpio_to_irq(pdata->row_gpios[i]), c2020_interrupt,
          IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "matrix-keypad", keypad);
      if (err < 0) {
        dev_err(&pdev->dev, "Unable to acquire interrupt for GPIO line %i\n",
                pdata->row_gpios[i]);
        goto err_free_irqs;
      }
    }
  }

  /* initialized as disabled - enabled by input->open */
  disable_row_irqs(keypad);
  return 0;

err_free_irqs:
  while (--i >= 0)
    free_irq(gpio_to_irq(pdata->row_gpios[i]), keypad);
  i = pdata->num_row_gpios;
err_free_rows:
  while (--i >= 0)
    gpio_free(pdata->row_gpios[i]);
  i = pdata->num_col_gpios;
err_free_cols:
  while (--i >= 0)
    gpio_free(pdata->col_gpios[i]);

  return err;
}

static void c2020_free_gpio(struct c2020 *keypad) {
  const struct c2020_platform_data *pdata = keypad->pdata;
  int i;

  if (pdata->clustered_irq > 0) {
    free_irq(pdata->clustered_irq, keypad);
  } else {
    for (i = 0; i < pdata->num_row_gpios; i++)
      free_irq(gpio_to_irq(pdata->row_gpios[i]), keypad);
  }

  for (i = 0; i < pdata->num_row_gpios; i++)
    gpio_free(pdata->row_gpios[i]);

  for (i = 0; i < pdata->num_col_gpios; i++)
    gpio_free(pdata->col_gpios[i]);
}

#ifdef CONFIG_OF
static struct c2020_platform_data *c2020_parse_dt(struct device *dev) {
  struct c2020_platform_data *pdata;
  struct device_node *np = dev->of_node;
  unsigned int *gpios;
  int ret, i, nrow, ncol;

  if (!np) {
    dev_err(dev, "device lacks DT data\n");
    return ERR_PTR(-ENODEV);
  }

  pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
  if (!pdata) {
    dev_err(dev, "could not allocate memory for platform data\n");
    return ERR_PTR(-ENOMEM);
  }

  pdata->num_row_gpios = nrow = of_gpio_named_count(np, "row-gpios");
  pdata->num_col_gpios = ncol = of_gpio_named_count(np, "col-gpios");
  if (nrow <= 0 || ncol <= 0) {
    dev_err(dev, "number of keypad rows/columns not specified\n");
    return ERR_PTR(-EINVAL);
  }

  if (of_get_property(np, "linux,no-autorepeat", NULL))
    pdata->no_autorepeat = true;

  pdata->wakeup = of_property_read_bool(np, "wakeup-source") ||
                  of_property_read_bool(np, "linux,wakeup"); /* legacy */

  if (of_get_property(np, "gpio-activelow", NULL))
    pdata->active_low = true;

  pdata->drive_inactive_cols = of_property_read_bool(np, "drive-inactive-cols");

  of_property_read_u32(np, "debounce-delay-ms", &pdata->debounce_ms);
  of_property_read_u32(np, "col-scan-delay-us", &pdata->col_scan_delay_us);
  of_property_read_u32(np, "modifier-scancycle-ms",
                       &pdata->modifier_scancycle_ms);

  printk("drive-inactive-cols: %d", pdata->drive_inactive_cols);

  gpios = devm_kcalloc(dev, pdata->num_row_gpios + pdata->num_col_gpios,
                       sizeof(unsigned int), GFP_KERNEL);
  if (!gpios) {
    dev_err(dev, "could not allocate memory for gpios\n");
    return ERR_PTR(-ENOMEM);
  }

  for (i = 0; i < nrow; i++) {
    ret = of_get_named_gpio(np, "row-gpios", i);
    if (ret < 0)
      return ERR_PTR(ret);
    gpios[i] = ret;
  }

  for (i = 0; i < ncol; i++) {
    ret = of_get_named_gpio(np, "col-gpios", i);
    if (ret < 0)
      return ERR_PTR(ret);
    gpios[nrow + i] = ret;
  }

  pdata->keycodes = kmalloc_array(pdata->num_row_gpios * pdata->num_col_gpios,
                              sizeof(u32), GFP_KERNEL);
  ret = of_property_read_u32_array(np, "linux,keymap", pdata->keycodes,
                             pdata->num_row_gpios * pdata->num_col_gpios);
  if (ret)
  {
    dev_err(dev, "Cannot read keymap from overlay. Wrong number of map entries?");
    return ERR_PTR(ret);
  }
  
  pdata->row_gpios = gpios;
  pdata->col_gpios = &gpios[pdata->num_row_gpios];

  return pdata;
}
#else
static inline struct c2020_platform_data *c2020_parse_dt(struct device *dev) {
  dev_err(dev, "no platform data defined\n");

  return ERR_PTR(-EINVAL);
}
#endif

static int c2020_probe(struct platform_device *pdev) {
  const struct c2020_platform_data *pdata;
  struct c2020 *keypad = NULL;
  struct input_dev *input_dev;
  int err;
  size_t max_keys;
  unsigned short *keymap;

  pdata = dev_get_platdata(&pdev->dev);
  if (!pdata) {
    pdata = c2020_parse_dt(&pdev->dev);
    if (IS_ERR(pdata))
      return PTR_ERR(pdata);
  } else if (!pdata->keymap_data) {
    dev_err(&pdev->dev, "no keymap data defined\n");
    return -EINVAL;
  }

  keypad = kzalloc(sizeof(struct c2020), GFP_KERNEL);
  input_dev = input_allocate_device();
  if (!keypad || !input_dev) {
    err = -ENOMEM;
    goto err_free_mem;
  }

  keypad->input_dev = input_dev;
  keypad->pdata = pdata;
  keypad->row_shift = get_count_order(pdata->num_col_gpios);
  keypad->stopped = true;
  INIT_DELAYED_WORK(&keypad->work, c2020_scan);
  spin_lock_init(&keypad->lock);

  max_keys = pdata->num_row_gpios << keypad->row_shift;

  input_dev->name = pdev->name;
  input_dev->id.bustype = BUS_HOST;
  input_dev->dev.parent = &pdev->dev;
  input_dev->open = c2020_start;
  input_dev->close = c2020_stop;

  __set_bit(EV_KEY, input_dev->evbit);

  keymap = devm_kcalloc(input_dev->dev.parent, max_keys, sizeof(*keymap),
                        GFP_KERNEL);

  if (!keymap) {
    dev_err(input_dev->dev.parent, "Unable to allocate memory for keymap");
    return -ENOMEM;
  }
  input_dev->keycode = keymap;
  input_dev->keycodesize = sizeof(*keymap);
  input_dev->keycodemax = max_keys;
  c2020_map_keys(input_dev, pdata);

  if (!pdata->no_autorepeat)
    __set_bit(EV_REP, input_dev->evbit);
  input_set_capability(input_dev, EV_MSC, MSC_SCAN);
  input_set_drvdata(input_dev, keypad);

  err = c2020_init_gpio(pdev, keypad);
  if (err)
    goto err_free_mem;

  err = input_register_device(keypad->input_dev);
  if (err)
    goto err_free_gpio;

  device_init_wakeup(&pdev->dev, pdata->wakeup);
  platform_set_drvdata(pdev, keypad);

  return 0;

err_free_gpio:
  c2020_free_gpio(keypad);
err_free_mem:
  input_free_device(input_dev);
  kfree(keypad);
  return err;
}

static int c2020_remove(struct platform_device *pdev) {
  struct c2020 *keypad = platform_get_drvdata(pdev);

  c2020_free_gpio(keypad);
  input_unregister_device(keypad->input_dev);
  kfree(keypad);

  return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id c2020_dt_match[] = {
    {.compatible = "gpio-c2020"}, {}};
MODULE_DEVICE_TABLE(of, c2020_dt_match);
#endif

static struct platform_driver c2020_driver = {
    .probe = c2020_probe,
    .remove = c2020_remove,
    .driver =
        {
            .name = "c2020",
            .pm = &c2020_pm_ops,
            .of_match_table = of_match_ptr(c2020_dt_match),
        },
};
module_platform_driver(c2020_driver);

MODULE_AUTHOR("Michael Linsenmeier <michalin70@gmail.com>");
MODULE_DESCRIPTION("GPIO Driven Matrix Keypad Driver with modifier key support");
MODULE_LICENSE("GPL v2");
// MODULE_ALIAS("platform:matrix-keypad");
