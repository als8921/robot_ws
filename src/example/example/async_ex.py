import asyncio

Menu = {
    "water" : 2,
    "chicken" : 5,
    "apple" : 3
}

async def order(name):
    await asyncio.sleep(Menu[name])
    print(f"{name}이 {Menu[name]}초가 걸려 나왔습니다.")

async def kiosk():
    await asyncio.gather(
        order("water"),
        order("water"),
        order("water"),
        order("chicken"),
        order("apple"),
    )


if __name__ == "__main__":
    asyncio.run(kiosk())