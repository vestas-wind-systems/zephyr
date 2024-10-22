/* buf_simple.c - Simple buffer management */

/*
 * Copyright (c) 2015-2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sys_buf_simple, CONFIG_SYS_BUF_LOG_LEVEL);

#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/sys_buf.h>

#if defined(CONFIG_SYS_BUF_SIMPLE_LOG)
#define SYS_BUF_SIMPLE_DBG(fmt, ...) LOG_DBG("(%p) " fmt, k_current_get(), \
				      ##__VA_ARGS__)
#define SYS_BUF_SIMPLE_ERR(fmt, ...) LOG_ERR(fmt, ##__VA_ARGS__)
#define SYS_BUF_SIMPLE_WARN(fmt, ...) LOG_WRN(fmt, ##__VA_ARGS__)
#define SYS_BUF_SIMPLE_INFO(fmt, ...) LOG_INF(fmt, ##__VA_ARGS__)
#else
#define SYS_BUF_SIMPLE_DBG(fmt, ...)
#define SYS_BUF_SIMPLE_ERR(fmt, ...)
#define SYS_BUF_SIMPLE_WARN(fmt, ...)
#define SYS_BUF_SIMPLE_INFO(fmt, ...)
#endif /* CONFIG_SYS_BUF_SIMPLE_LOG */

void sys_buf_simple_init_with_data(struct sys_buf_simple *buf,
				   void *data, size_t size)
{
	buf->__buf = data;
	buf->data  = data;
	buf->size  = size;
	buf->len   = size;
}

void sys_buf_simple_reserve(struct sys_buf_simple *buf, size_t reserve)
{
	__ASSERT_NO_MSG(buf);
	__ASSERT_NO_MSG(buf->len == 0U);
	SYS_BUF_SIMPLE_DBG("buf %p reserve %zu", buf, reserve);

	buf->data = buf->__buf + reserve;
}

void sys_buf_simple_clone(const struct sys_buf_simple *original,
			  struct sys_buf_simple *clone)
{
	memcpy(clone, original, sizeof(struct sys_buf_simple));
}

void *sys_buf_simple_add(struct sys_buf_simple *buf, size_t len)
{
	uint8_t *tail = sys_buf_simple_tail(buf);

	SYS_BUF_SIMPLE_DBG("buf %p len %zu", buf, len);

	__ASSERT_NO_MSG(sys_buf_simple_tailroom(buf) >= len);

	buf->len += len;
	return tail;
}

void *sys_buf_simple_add_mem(struct sys_buf_simple *buf, const void *mem,
			     size_t len)
{
	SYS_BUF_SIMPLE_DBG("buf %p len %zu", buf, len);

	return memcpy(sys_buf_simple_add(buf, len), mem, len);
}

uint8_t *sys_buf_simple_add_u8(struct sys_buf_simple *buf, uint8_t val)
{
	uint8_t *u8;

	SYS_BUF_SIMPLE_DBG("buf %p val 0x%02x", buf, val);

	u8 = sys_buf_simple_add(buf, 1);
	*u8 = val;

	return u8;
}

void sys_buf_simple_add_le16(struct sys_buf_simple *buf, uint16_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_le16(val, sys_buf_simple_add(buf, sizeof(val)));
}

void sys_buf_simple_add_be16(struct sys_buf_simple *buf, uint16_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_be16(val, sys_buf_simple_add(buf, sizeof(val)));
}

void sys_buf_simple_add_le24(struct sys_buf_simple *buf, uint32_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_le24(val, sys_buf_simple_add(buf, 3));
}

void sys_buf_simple_add_be24(struct sys_buf_simple *buf, uint32_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_be24(val, sys_buf_simple_add(buf, 3));
}

void sys_buf_simple_add_le32(struct sys_buf_simple *buf, uint32_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_le32(val, sys_buf_simple_add(buf, sizeof(val)));
}

void sys_buf_simple_add_be32(struct sys_buf_simple *buf, uint32_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_be32(val, sys_buf_simple_add(buf, sizeof(val)));
}

void sys_buf_simple_add_le40(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_le40(val, sys_buf_simple_add(buf, 5));
}

void sys_buf_simple_add_be40(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_be40(val, sys_buf_simple_add(buf, 5));
}

void sys_buf_simple_add_le48(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_le48(val, sys_buf_simple_add(buf, 6));
}

void sys_buf_simple_add_be48(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_be48(val, sys_buf_simple_add(buf, 6));
}

void sys_buf_simple_add_le64(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_le64(val, sys_buf_simple_add(buf, sizeof(val)));
}

void sys_buf_simple_add_be64(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_be64(val, sys_buf_simple_add(buf, sizeof(val)));
}

void *sys_buf_simple_remove_mem(struct sys_buf_simple *buf, size_t len)
{
	SYS_BUF_SIMPLE_DBG("buf %p len %zu", buf, len);

	__ASSERT_NO_MSG(buf->len >= len);

	buf->len -= len;
	return buf->data + buf->len;
}

uint8_t sys_buf_simple_remove_u8(struct sys_buf_simple *buf)
{
	uint8_t val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = *(uint8_t *)ptr;

	return val;
}

uint16_t sys_buf_simple_remove_le16(struct sys_buf_simple *buf)
{
	uint16_t val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((uint16_t *)ptr);

	return sys_le16_to_cpu(val);
}

uint16_t sys_buf_simple_remove_be16(struct sys_buf_simple *buf)
{
	uint16_t val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((uint16_t *)ptr);

	return sys_be16_to_cpu(val);
}

uint32_t sys_buf_simple_remove_le24(struct sys_buf_simple *buf)
{
	struct uint24 {
		uint32_t u24 : 24;
	} __packed val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((struct uint24 *)ptr);

	return sys_le24_to_cpu(val.u24);
}

uint32_t sys_buf_simple_remove_be24(struct sys_buf_simple *buf)
{
	struct uint24 {
		uint32_t u24 : 24;
	} __packed val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((struct uint24 *)ptr);

	return sys_be24_to_cpu(val.u24);
}

uint32_t sys_buf_simple_remove_le32(struct sys_buf_simple *buf)
{
	uint32_t val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((uint32_t *)ptr);

	return sys_le32_to_cpu(val);
}

uint32_t sys_buf_simple_remove_be32(struct sys_buf_simple *buf)
{
	uint32_t val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((uint32_t *)ptr);

	return sys_be32_to_cpu(val);
}

uint64_t sys_buf_simple_remove_le40(struct sys_buf_simple *buf)
{
	struct uint40 {
		uint64_t u40: 40;
	} __packed val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((struct uint40 *)ptr);

	return sys_le40_to_cpu(val.u40);
}

uint64_t sys_buf_simple_remove_be40(struct sys_buf_simple *buf)
{
	struct uint40 {
		uint64_t u40: 40;
	} __packed val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((struct uint40 *)ptr);

	return sys_be40_to_cpu(val.u40);
}

uint64_t sys_buf_simple_remove_le48(struct sys_buf_simple *buf)
{
	struct uint48 {
		uint64_t u48 : 48;
	} __packed val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((struct uint48 *)ptr);

	return sys_le48_to_cpu(val.u48);
}

uint64_t sys_buf_simple_remove_be48(struct sys_buf_simple *buf)
{
	struct uint48 {
		uint64_t u48 : 48;
	} __packed val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((struct uint48 *)ptr);

	return sys_be48_to_cpu(val.u48);
}

uint64_t sys_buf_simple_remove_le64(struct sys_buf_simple *buf)
{
	uint64_t val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((uint64_t *)ptr);

	return sys_le64_to_cpu(val);
}

uint64_t sys_buf_simple_remove_be64(struct sys_buf_simple *buf)
{
	uint64_t val;
	void *ptr;

	ptr = sys_buf_simple_remove_mem(buf, sizeof(val));
	val = UNALIGNED_GET((uint64_t *)ptr);

	return sys_be64_to_cpu(val);
}

void *sys_buf_simple_push(struct sys_buf_simple *buf, size_t len)
{
	SYS_BUF_SIMPLE_DBG("buf %p len %zu", buf, len);

	__ASSERT_NO_MSG(sys_buf_simple_headroom(buf) >= len);

	buf->data -= len;
	buf->len += len;
	return buf->data;
}

void *sys_buf_simple_push_mem(struct sys_buf_simple *buf, const void *mem,
			      size_t len)
{
	SYS_BUF_SIMPLE_DBG("buf %p len %zu", buf, len);

	return memcpy(sys_buf_simple_push(buf, len), mem, len);
}

void sys_buf_simple_push_le16(struct sys_buf_simple *buf, uint16_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_le16(val, sys_buf_simple_push(buf, sizeof(val)));
}

void sys_buf_simple_push_be16(struct sys_buf_simple *buf, uint16_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_be16(val, sys_buf_simple_push(buf, sizeof(val)));
}

void sys_buf_simple_push_u8(struct sys_buf_simple *buf, uint8_t val)
{
	uint8_t *data = sys_buf_simple_push(buf, 1);

	*data = val;
}

void sys_buf_simple_push_le24(struct sys_buf_simple *buf, uint32_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_le24(val, sys_buf_simple_push(buf, 3));
}

void sys_buf_simple_push_be24(struct sys_buf_simple *buf, uint32_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_be24(val, sys_buf_simple_push(buf, 3));
}

void sys_buf_simple_push_le32(struct sys_buf_simple *buf, uint32_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_le32(val, sys_buf_simple_push(buf, sizeof(val)));
}

void sys_buf_simple_push_be32(struct sys_buf_simple *buf, uint32_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %u", buf, val);

	sys_put_be32(val, sys_buf_simple_push(buf, sizeof(val)));
}

void sys_buf_simple_push_le40(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_le40(val, sys_buf_simple_push(buf, 5));
}

void sys_buf_simple_push_be40(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_be40(val, sys_buf_simple_push(buf, 5));
}

void sys_buf_simple_push_le48(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_le48(val, sys_buf_simple_push(buf, 6));
}

void sys_buf_simple_push_be48(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_be48(val, sys_buf_simple_push(buf, 6));
}

void sys_buf_simple_push_le64(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_le64(val, sys_buf_simple_push(buf, sizeof(val)));
}

void sys_buf_simple_push_be64(struct sys_buf_simple *buf, uint64_t val)
{
	SYS_BUF_SIMPLE_DBG("buf %p val %" PRIu64, buf, val);

	sys_put_be64(val, sys_buf_simple_push(buf, sizeof(val)));
}

void *sys_buf_simple_pull(struct sys_buf_simple *buf, size_t len)
{
	SYS_BUF_SIMPLE_DBG("buf %p len %zu", buf, len);

	__ASSERT_NO_MSG(buf->len >= len);

	buf->len -= len;
	return buf->data += len;
}

void *sys_buf_simple_pull_mem(struct sys_buf_simple *buf, size_t len)
{
	void *data = buf->data;

	SYS_BUF_SIMPLE_DBG("buf %p len %zu", buf, len);

	__ASSERT_NO_MSG(buf->len >= len);

	buf->len -= len;
	buf->data += len;

	return data;
}

uint8_t sys_buf_simple_pull_u8(struct sys_buf_simple *buf)
{
	uint8_t val;

	val = buf->data[0];
	sys_buf_simple_pull(buf, 1);

	return val;
}

uint16_t sys_buf_simple_pull_le16(struct sys_buf_simple *buf)
{
	uint16_t val;

	val = UNALIGNED_GET((uint16_t *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_le16_to_cpu(val);
}

uint16_t sys_buf_simple_pull_be16(struct sys_buf_simple *buf)
{
	uint16_t val;

	val = UNALIGNED_GET((uint16_t *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_be16_to_cpu(val);
}

uint32_t sys_buf_simple_pull_le24(struct sys_buf_simple *buf)
{
	struct uint24 {
		uint32_t u24:24;
	} __packed val;

	val = UNALIGNED_GET((struct uint24 *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_le24_to_cpu(val.u24);
}

uint32_t sys_buf_simple_pull_be24(struct sys_buf_simple *buf)
{
	struct uint24 {
		uint32_t u24:24;
	} __packed val;

	val = UNALIGNED_GET((struct uint24 *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_be24_to_cpu(val.u24);
}

uint32_t sys_buf_simple_pull_le32(struct sys_buf_simple *buf)
{
	uint32_t val;

	val = UNALIGNED_GET((uint32_t *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_le32_to_cpu(val);
}

uint32_t sys_buf_simple_pull_be32(struct sys_buf_simple *buf)
{
	uint32_t val;

	val = UNALIGNED_GET((uint32_t *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_be32_to_cpu(val);
}

uint64_t sys_buf_simple_pull_le40(struct sys_buf_simple *buf)
{
	struct uint40 {
		uint64_t u40: 40;
	} __packed val;

	val = UNALIGNED_GET((struct uint40 *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_le40_to_cpu(val.u40);
}

uint64_t sys_buf_simple_pull_be40(struct sys_buf_simple *buf)
{
	struct uint40 {
		uint64_t u40: 40;
	} __packed val;

	val = UNALIGNED_GET((struct uint40 *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_be40_to_cpu(val.u40);
}

uint64_t sys_buf_simple_pull_le48(struct sys_buf_simple *buf)
{
	struct uint48 {
		uint64_t u48:48;
	} __packed val;

	val = UNALIGNED_GET((struct uint48 *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_le48_to_cpu(val.u48);
}

uint64_t sys_buf_simple_pull_be48(struct sys_buf_simple *buf)
{
	struct uint48 {
		uint64_t u48:48;
	} __packed val;

	val = UNALIGNED_GET((struct uint48 *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_be48_to_cpu(val.u48);
}

uint64_t sys_buf_simple_pull_le64(struct sys_buf_simple *buf)
{
	uint64_t val;

	val = UNALIGNED_GET((uint64_t *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_le64_to_cpu(val);
}

uint64_t sys_buf_simple_pull_be64(struct sys_buf_simple *buf)
{
	uint64_t val;

	val = UNALIGNED_GET((uint64_t *)buf->data);
	sys_buf_simple_pull(buf, sizeof(val));

	return sys_be64_to_cpu(val);
}

size_t sys_buf_simple_headroom(const struct sys_buf_simple *buf)
{
	return buf->data - buf->__buf;
}

size_t sys_buf_simple_tailroom(const struct sys_buf_simple *buf)
{
	return buf->size - sys_buf_simple_headroom(buf) - buf->len;
}

uint16_t sys_buf_simple_max_len(const struct sys_buf_simple *buf)
{
	return buf->size - sys_buf_simple_headroom(buf);
}
